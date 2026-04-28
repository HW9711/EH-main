//uart2.c

#include "main.h"
#include "uart2.h"
#include "bsp_uart.h"
#include "common.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART2_TimeoutComp   3
/* 打开 UART2 发送诊断开关后，每次 UART2 发送完成都会从 UART10 输出 HAL 返回值。 */
#define UART2_TX_TRACE_ENABLE  1U
/* UART10 诊断发送超时时间，避免调试打印长时间占用 AppTask。 */
#define UART2_TX_TRACE_TIMEOUT 100U
/* UART2 发送改为中断方式，避免 HAL_UART_Transmit 阻塞 AppTask 100ms。 */
#define UART2_TX_USE_INTERRUPT 1U
/* UART2 中断发送超过该时间仍未完成时，下次发送前主动恢复 USART2 外设。 */
#define UART2_TX_BUSY_RECOVER_MS 20U
/* UART2 诊断恢复时使用当前句柄波特率异常时的兜底波特率。 */
#define UART2_RECOVER_FALLBACK_BAUD 115200U
/* UART2 RX 侧真实异常标志，只把 PE/FE/NE/ORE 当作需要恢复接收 DMA 的错误。 */
#define UART2_RX_FAULT_SR_MASK (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)
/* UART2 发送寄存器空标志，正常空闲时该位必须为 1。 */
#define UART2_TX_READY_SR_MASK USART_SR_TXE
/* UART2 主动恢复原因：发送前发现 RX/TX 状态异常。 */
#define UART2_RECOVER_REASON_PRECHECK 1U
/* UART2 主动恢复原因：启动中断发送接口返回失败。 */
#define UART2_RECOVER_REASON_START_FAIL 2U
/* UART2 主动恢复原因：上一包中断发送长时间未完成。 */
#define UART2_RECOVER_REASON_BUSY_TIMEOUT 3U
/* UART2 主动恢复原因：HAL UART 错误回调通知 USART2 出错。 */
#define UART2_RECOVER_REASON_ERROR_CALLBACK 4U
/* UART2 主动恢复原因：发送前只发现 RX 侧奇偶校验、帧错误、噪声、溢出或 DMA 接收异常。 */
#define UART2_RECOVER_REASON_RX_PRECHECK 5U
/* UART2 跳过发送原因：完整恢复刚执行完，本帧不继续占用可能尚未稳定的 TX 引脚。 */
#define UART2_TX_SKIP_REASON_AFTER_FULL_RECOVER 1U
/* UART2 跳过发送原因：发送前检查发现 USART2 的 TXE/TE/UE 条件仍不满足。 */
#define UART2_TX_SKIP_REASON_TX_NOT_READY 2U

static uint8_t Uart2_Flag_Last = 0;
static uint16_t Uart2_RecvWaitTimeCnt = 0;
static uint8_t Uart2_DMABuf[UART2_MAX_PACKET_SIZE] = { 0 };
/* UART2 中断发送专用缓存，避免调用方复用 s_tx_buf 时破坏尚未发完的数据。 */
static uint8_t Uart2_TXBuf[UART2_MAX_PACKET_SIZE] = { 0 };
/* UART2 中断发送忙标志，置 1 表示上一包已经交给 HAL_UART_Transmit_IT。 */
static volatile uint8_t Uart2_TxBusy = 0U;
/* UART2 中断发送开始时间，用于判断 TX 完成回调是否异常丢失。 */
static volatile uint32_t Uart2_TxStartTick = 0U;
/* UART2 错误回调置位该标志，下一次任务上下文发送前再做完整恢复。 */
static volatile uint8_t Uart2_NeedRecover = 0U;

static void Uart2_DebugTraceRecover(uint8_t reason,
                                    HAL_StatusTypeDef status,
                                    uint32_t sr_before,
                                    uint32_t er_before,
                                    uint32_t sr_after)
{
#if (UART2_TX_TRACE_ENABLE == 1U)
  /* tx_buf 保存 UART2 外设恢复诊断报文。 */
  char tx_buf[96];
  /* text_len 保存 snprintf 生成的实际字符数。 */
  int text_len;

  /* 输出恢复原因、恢复结果、恢复前 SR/ER 和恢复后 SR，便于判断是否清掉错误位。 */
  text_len = snprintf(tx_buf,
                      sizeof(tx_buf),
                      "U2RCV,RS=%02u,ST=%02u,SB=%08lX,ER=%08lX,SA=%08lX\r\n",
                      (unsigned int)reason,
                      (unsigned int)status,
                      (unsigned long)sr_before,
                      (unsigned long)er_before,
                      (unsigned long)sr_after);
  if (text_len <= 0)
  {
    /* 文本生成失败时不输出，避免发送未初始化内容。 */
    return;
  }

  if ((size_t)text_len > (sizeof(tx_buf) - 1U))
  {
    /* snprintf 发生截断时，按缓冲区最大有效长度发送。 */
    text_len = (int)(sizeof(tx_buf) - 1U);
  }

  /* 通过 UART10 输出恢复诊断，和 U2TX 发送诊断分开。 */
  (void)Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, UART2_TX_TRACE_TIMEOUT);
#else
  /* 关闭诊断时丢弃所有参数，保证无警告编译。 */
  (void)reason;
  (void)status;
  (void)sr_before;
  (void)er_before;
  (void)sr_after;
#endif
}

static void Uart2_DebugTraceSkip(uint8_t reason, uint16_t length)
{
#if (UART2_TX_TRACE_ENABLE == 1U)
  /* uart2 指向 USART2 句柄，用于把跳过发送时的 HAL 状态和寄存器状态一起打印出来。 */
  UART_HandleTypeDef *uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  /* tx_buf 保存 UART10 跳过发送诊断文本。 */
  char tx_buf[96];
  /* text_len 保存 snprintf 实际写入的字符数。 */
  int text_len;
  /* gstate 保存 UART2 当前发送状态机，句柄不可用时用 0xFF 标识。 */
  uint8_t gstate = 0xFFU;
  /* error_code 保存 HAL UART 错误码，便于确认是否还有 FE/NE/ORE 残留。 */
  uint32_t error_code = 0U;
  /* sr_value 保存 USART2 状态寄存器，便于确认 TXE/TC/RXNE/LBD 等位。 */
  uint32_t sr_value = 0U;

  if (uart2 != NULL)
  {
    /* 读取 HAL 发送状态，判断跳过时是否仍处在 BUSY_TX。 */
    gstate = (uint8_t)uart2->gState;
    /* 读取 HAL 错误码，判断跳过是否由 RX 错误牵连触发。 */
    error_code = uart2->ErrorCode;
    /* 读取 USART2 SR，现场可直接和 U2RCV 的 SA 字段对照。 */
    sr_value = uart2->Instance->SR;
  }

  /* U2SKP 表示本次心跳已经被驱动主动丢弃，没有交给 HAL_UART_Transmit_IT。 */
  text_len = snprintf(tx_buf,
                      sizeof(tx_buf),
                      "U2SKP,RS=%02u,LN=%04u,GS=%02u,ER=%08lX,SR=%08lX\r\n",
                      (unsigned int)reason,
                      (unsigned int)length,
                      (unsigned int)gstate,
                      (unsigned long)error_code,
                      (unsigned long)sr_value);
  if (text_len <= 0)
  {
    /* 文本生成失败时不输出，避免 UART10 打印无效内存。 */
    return;
  }

  if ((size_t)text_len > (sizeof(tx_buf) - 1U))
  {
    /* snprintf 截断时按缓冲区最大有效长度发送，防止越界。 */
    text_len = (int)(sizeof(tx_buf) - 1U);
  }

  /* 通过 UART10 打印跳过原因，帮助确认是否成功挡住 SA=00000000 后继续发送。 */
  (void)Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, UART2_TX_TRACE_TIMEOUT);
#else
  /* 关闭诊断时丢弃参数，避免未使用警告。 */
  (void)reason;
  (void)length;
#endif
}

static void Uart2_ClearRxFaultFlags(UART_HandleTypeDef *uart2)
{
  /* tmpreg 用于执行 STM32F4 清 FE/NE/ORE/PE/IDLE 的 SR 后 DR 读取序列。 */
  __IO uint32_t tmpreg;

  if ((uart2 == NULL) || (uart2->Instance == NULL))
  {
    /* UART2 句柄不可用时无法清标志，直接返回。 */
    return;
  }

  /* 先读 SR 再读 DR，清除 FE/NE/ORE/PE/IDLE，同时释放 RXNE。 */
  tmpreg = uart2->Instance->SR;
  tmpreg = uart2->Instance->DR;
  UNUSED(tmpreg);

  /* LBD 需要按 SR 写 0 的方式单独清掉，避免 Break 标志残留。 */
  __HAL_UART_CLEAR_FLAG(uart2, UART_FLAG_LBD);
}

static void Uart2_RestartRxDmaBuffer(void)
{
  /* 先停止 UART2 RX DMA，防止旧 DMA 状态继续写入旧缓冲。 */
  (void)Bsp_UartDmaStop(BSP_UART_PORT_2);
  /* 清空 UART2 接收缓存，避免恢复后解析到错误帧残留。 */
  memset(Uart2_DMABuf, 0, UART2_MAX_PACKET_SIZE);
  /* 重新启动 UART2 RX DMA，恢复外部通信下行接收能力。 */
  (void)Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
  /* 清零静默计数，避免恢复后立刻把旧状态当成一包完整数据。 */
  Uart2_RecvWaitTimeCnt = 0U;
  /* 复位 DMA 剩余长度快照，让下一次 Peek 从完整空缓存开始判断。 */
  Uart2_Flag_Last = UART2_MAX_PACKET_SIZE;
}

static uint8_t Uart2_IsTxHardwareReady(UART_HandleTypeDef *uart2, uint32_t *sr_out)
{
  /* sr_value 保存 USART2 状态寄存器，默认 0 表示不可用或尚未读取。 */
  uint32_t sr_value = 0U;
  /* cr1_value 保存 USART2 控制寄存器 1，用于确认 UE 和 TE 是否已经打开。 */
  uint32_t cr1_value = 0U;

  if ((uart2 == NULL) || (uart2->Instance == NULL))
  {
    /* 句柄不可用时，调用者拿到 0，并把 TX 当作未就绪处理。 */
    if (sr_out != NULL)
    {
      /* 输出 0，便于诊断中明确区分“未读到寄存器”和“TXE 正常”。 */
      *sr_out = 0U;
    }
    return 0U;
  }

  /* 读取 SR，重点看 TXE 是否为 1。 */
  sr_value = uart2->Instance->SR;
  /* 读取 CR1，确认 USART 使能 UE 和发送使能 TE 没有被恢复流程关掉。 */
  cr1_value = uart2->Instance->CR1;

  if (sr_out != NULL)
  {
    /* 把 SR 回传给调用者，避免调用者为了诊断再次读寄存器。 */
    *sr_out = sr_value;
  }

  if ((cr1_value & USART_CR1_UE) == 0U)
  {
    /* UE 没开时 USART2 外设整体未使能，不能启动中断发送。 */
    return 0U;
  }

  if ((cr1_value & USART_CR1_TE) == 0U)
  {
    /* TE 没开时 TX 引脚不会由 USART2 正常驱动，继续发送容易在线上形成乱码。 */
    return 0U;
  }

  if ((sr_value & UART2_TX_READY_SR_MASK) == 0U)
  {
    /* TXE 为 0 表示发送数据寄存器还不可写，空闲心跳不应强行启动新发送。 */
    return 0U;
  }

  /* UE、TE、TXE 均满足，认为硬件发送通路可以交给 HAL_UART_Transmit_IT。 */
  return 1U;
}

static HAL_StatusTypeDef Uart2_RecoverRxPath(uint8_t reason)
{
  /* uart2 指向 USART2 HAL 句柄，RX-only 恢复不改变 TX 引脚复用配置。 */
  UART_HandleTypeDef *uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  /* sr_before 保存恢复前 SR，用来确认是否存在 FE/NE/ORE/RXNE/LBD。 */
  uint32_t sr_before = 0U;
  /* er_before 保存恢复前 HAL 错误码，用来确认 HAL_UART_ErrorCallback 的来源。 */
  uint32_t er_before = 0U;
  /* sr_after 保存恢复后 SR，用来判断 RX 错误位是否已经被清理。 */
  uint32_t sr_after = 0U;
  /* status 保存 RX DMA 重新启动结果。 */
  HAL_StatusTypeDef status = HAL_OK;

  if (uart2 == NULL)
  {
    /* 句柄不存在时无法做 RX-only 恢复，直接输出错误诊断。 */
    Uart2_DebugTraceRecover(reason, HAL_ERROR, 0U, 0U, 0U);
    return HAL_ERROR;
  }

  /* 记录恢复前 SR，现场可判断是不是只有 RX 噪声触发。 */
  sr_before = uart2->Instance->SR;
  /* 记录恢复前 HAL 错误码，常见为 FE/NE/ORE。 */
  er_before = uart2->ErrorCode;
  /* 清掉任务上下文恢复请求，避免下一帧重复做同一轮 RX 恢复。 */
  Uart2_NeedRecover = 0U;
  /* 先停止并重启 RX DMA，只处理接收通路，不反初始化 USART2。 */
  Uart2_RestartRxDmaBuffer();
  /* 重启 DMA 后再执行一次 SR/DR 清标志，清掉恢复过程中残留的 RXNE/IDLE。 */
  Uart2_ClearRxFaultFlags(uart2);
  /* RX 错误已经在本轮处理，清掉 HAL 错误码，避免后续 U2TX 持续显示旧错误。 */
  uart2->ErrorCode = HAL_UART_ERROR_NONE;
  /* 如果 RX DMA 没能启动，RxState 可能不是 BUSY_RX；这里用状态判断恢复结果。 */
  if (uart2->RxState != HAL_UART_STATE_BUSY_RX)
  {
    /* RX 未回到 BUSY_RX，说明接收 DMA 没有恢复到持续接收状态。 */
    status = HAL_ERROR;
  }
  /* 记录恢复后的 SR，确认 TXE/TC 是否保持、RX 错误是否消失。 */
  sr_after = uart2->Instance->SR;
  /* 输出 RX-only 恢复诊断，注意此恢复不会让 UART2 TX 引脚悬空。 */
  Uart2_DebugTraceRecover(reason, status, sr_before, er_before, sr_after);
  return status;
}

static HAL_StatusTypeDef Uart2_RecoverPeripheral(uint8_t reason)
{
  /* uart2 指向 USART2 HAL 句柄，恢复流程需要读取当前配置和寄存器。 */
  UART_HandleTypeDef *uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  /* baud 保存恢复前的波特率，避免重初始化后串口参数漂移。 */
  uint32_t baud = UART2_RECOVER_FALLBACK_BAUD;
  /* sr_before 保存恢复前 USART2 状态寄存器，供 UART10 诊断输出。 */
  uint32_t sr_before = 0U;
  /* er_before 保存恢复前 HAL 错误码，供 UART10 诊断输出。 */
  uint32_t er_before = 0U;
  /* sr_after 保存恢复后 USART2 状态寄存器，供 UART10 诊断输出。 */
  uint32_t sr_after = 0U;
  /* status 保存恢复流程最终结果。 */
  HAL_StatusTypeDef status = HAL_OK;

  if (uart2 == NULL)
  {
    /* 句柄不存在时恢复失败，只能输出 HAL_ERROR。 */
    Uart2_DebugTraceRecover(reason, HAL_ERROR, 0U, 0U, 0U);
    return HAL_ERROR;
  }

  /* 记录恢复前 SR，用于确认是否存在 FE/NE/ORE/LBD 或 TXE 异常。 */
  sr_before = uart2->Instance->SR;
  /* 记录恢复前 HAL 错误码，用于确认 FE/NE/ORE 等 HAL 错误来源。 */
  er_before = uart2->ErrorCode;
  /* 如果句柄中保存了有效波特率，就沿用当前配置。 */
  if (uart2->Init.BaudRate != 0U)
  {
    baud = uart2->Init.BaudRate;
  }

  /* 恢复开始时先认为 TX 不忙，避免旧中断发送状态阻塞后续心跳。 */
  Uart2_TxBusy = 0U;
  /* 清掉延迟恢复标志，当前任务上下文马上执行完整恢复。 */
  Uart2_NeedRecover = 0U;

  /* 停止 UART2 当前收发过程，清掉 HAL 内部 BUSY_TX/BUSY_RX 状态。 */
  (void)Bsp_UartAbort(BSP_UART_PORT_2);
  /* 清除 RX 侧错误标志，避免 FE/NE/ORE/LBD 残留影响后续 USART2 中断。 */
  Uart2_ClearRxFaultFlags(uart2);
  /* 反初始化 USART2，重新走一次 MSP GPIO/DMA/NVIC 配置。 */
  if (Bsp_UartDeInit(BSP_UART_PORT_2) != HAL_OK)
  {
    /* DeInit 失败时标记恢复失败，但仍继续尝试 Init。 */
    status = HAL_ERROR;
  }
  /* 按原波特率重新初始化 USART2。 */
  if (Bsp_UartInit(BSP_UART_PORT_2, baud) != HAL_OK)
  {
    /* Init 失败时无法可靠恢复 UART2，输出诊断后返回。 */
    status = HAL_ERROR;
  }
  else
  {
    /* USART2 初始化成功后，重启 RX DMA 接收缓存。 */
    Uart2_RestartRxDmaBuffer();
  }

  /* 重新获取句柄，防止初始化过程更新了 HAL 内部状态。 */
  uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  if (uart2 != NULL)
  {
    /* 完整重初始化后再清一次 RX 标志，防止恢复窗口内的新噪声残留到下一帧。 */
    Uart2_ClearRxFaultFlags(uart2);
    /* 清掉 HAL 错误码，避免旧的 FE/NE/ORE 让后续诊断误判为新错误。 */
    uart2->ErrorCode = HAL_UART_ERROR_NONE;
    /* 记录恢复后的 SR，用于判断 TXE/TC 是否已经恢复正常。 */
    sr_after = uart2->Instance->SR;
    if (Uart2_IsTxHardwareReady(uart2, &sr_after) == 0U)
    {
      /* 如果完整恢复后 TXE/TE/UE 仍未满足，本轮恢复只能视为暂未稳定。 */
      status = HAL_BUSY;
    }
  }

  /* 通过 UART10 输出恢复诊断。 */
  Uart2_DebugTraceRecover(reason, status, sr_before, er_before, sr_after);
  return status;
}

static void Uart2_DebugTraceTx(HAL_StatusTypeDef status, uint16_t length)
{
#if (UART2_TX_TRACE_ENABLE == 1U)
  /* uart2 指向 USART2 句柄，用于读取 HAL 状态机和错误码。 */
  UART_HandleTypeDef *uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  /* tx_buf 保存 UART10 文本诊断报文，长度只放一行短文本。 */
  char tx_buf[96];
  /* text_len 保存 snprintf 返回的实际文本长度。 */
  int text_len;
  /* gstate 保存 UART2 发送状态机值，句柄为空时用 0xFF 表示不可读。 */
  uint8_t gstate = 0xFFU;
  /* error_code 保存 HAL_UART_GetError 对应的错误位图，句柄为空时保持 0。 */
  uint32_t error_code = 0U;
  /* sr_value 保存 USART2 状态寄存器，句柄为空时保持 0。 */
  uint32_t sr_value = 0U;

  if (uart2 != NULL)
  {
    /* 读取 HAL 全局状态，正常空闲通常为 HAL_UART_STATE_READY。 */
    gstate = (uint8_t)uart2->gState;
    /* 读取 HAL 错误码，用于判断是否有 ORE/FE/NE/PE 等 UART 错误。 */
    error_code = uart2->ErrorCode;
    /* 读取 USART2 SR，重点观察 TXE/TC 是否异常。 */
    sr_value = uart2->Instance->SR;
  }

  /* 组装 UART10 诊断文本：ST 是 HAL_UART_Transmit 返回值，LN 是本次发送长度。 */
  text_len = snprintf(tx_buf,
                      sizeof(tx_buf),
                      "U2TX,ST=%02u,LN=%04u,GS=%02u,ER=%08lX,SR=%08lX\r\n",
                      (unsigned int)status,
                      (unsigned int)length,
                      (unsigned int)gstate,
                      (unsigned long)error_code,
                      (unsigned long)sr_value);
  if (text_len <= 0)
  {
    /* snprintf 失败时不再发送，避免 UART10 打出无效内容。 */
    return;
  }

  if ((size_t)text_len > (sizeof(tx_buf) - 1U))
  {
    /* 文本被截断时按缓冲区最大有效长度发送，防止越界。 */
    text_len = (int)(sizeof(tx_buf) - 1U);
  }

  /* 通过 UART10 输出诊断报文，不影响 UART2 当前发送返回值本身。 */
  (void)Bsp_UartTransmit(BSP_UART_PORT_10, (uint8_t *)tx_buf, (uint16_t)text_len, UART2_TX_TRACE_TIMEOUT);
#else
  /* 关闭诊断时显式丢弃参数，避免编译器产生未使用警告。 */
  (void)status;
  (void)length;
#endif
}

static void Uart2_DMAConfiguration(void)
{
//	Delay_ms(300);

  Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE);
}

void Uart2_Configuration(uint16_t baud)
{
  if (Bsp_UartInit(BSP_UART_PORT_2, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart2_DMAReset(void)
{
  Uart2_RestartRxDmaBuffer();
}

void Uart2_Init(void)
{
  Uart2_DMAConfiguration();
}

void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
#if (UART2_TX_USE_INTERRUPT == 1U)
  /* uart2 指向 USART2 HAL 句柄，中断发送和状态检查都基于该句柄。 */
  UART_HandleTypeDef *uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
  /* status 保存 HAL_UART_Transmit_IT 的启动返回值。 */
  HAL_StatusTypeDef status;
  /* now_tick 保存当前系统 tick，用于判断上一包中断发送是否超时。 */
  uint32_t now_tick;
  /* sr_value 保存发送前 USART2 状态寄存器，用于判断是否需要先恢复外设。 */
  uint32_t sr_value;

  if ((uart2 == NULL) || (pData == NULL) || (Length == 0U) || (Length > UART2_MAX_PACKET_SIZE))
  {
    /* 参数异常时不启动发送，只输出 HAL_ERROR 诊断。 */
    Uart2_DebugTraceTx(HAL_ERROR, Length);
    return;
  }

  /* 当前 tick 用于 TX 忙超时判断。 */
  now_tick = HAL_GetTick();
  if (Uart2_TxBusy != 0U)
  {
    if (uart2->gState == HAL_UART_STATE_READY)
    {
      /* HAL 已经回到 READY，说明完成回调可能丢失，这里补清忙标志。 */
      Uart2_TxBusy = 0U;
    }
    else if ((uint32_t)(now_tick - Uart2_TxStartTick) > UART2_TX_BUSY_RECOVER_MS)
    {
      /* 上一包超过 20ms 未结束，判定中断发送异常，先恢复 USART2。 */
      (void)Uart2_RecoverPeripheral(UART2_RECOVER_REASON_BUSY_TIMEOUT);
      /* 完整恢复会重新配置 TX/RX 引脚，本帧先跳过，下一次心跳再进入正常发送。 */
      Uart2_DebugTraceSkip(UART2_TX_SKIP_REASON_AFTER_FULL_RECOVER, Length);
      return;
    }
    else
    {
      /* 上一包仍在合理发送窗口内，本包直接丢弃并输出 HAL_BUSY。 */
      Uart2_DebugTraceTx(HAL_BUSY, Length);
      return;
    }
  }

  /* 读取发送前 SR，先区分 RX 噪声和 TX 硬件未就绪。 */
  sr_value = uart2->Instance->SR;
  if ((Uart2_NeedRecover != 0U) ||
      ((sr_value & UART2_RX_FAULT_SR_MASK) != 0U))
  {
    /* 只发现 RX 侧错误时，仅重启 RX DMA 和清错误位，不反初始化 USART2。 */
    (void)Uart2_RecoverRxPath(UART2_RECOVER_REASON_RX_PRECHECK);
    /* RX-only 恢复后重新读取句柄，保证后续发送使用最新 HAL 状态。 */
    uart2 = Bsp_UartHandle(BSP_UART_PORT_2);
    if (uart2 == NULL)
    {
      /* 恢复后句柄仍不可用时输出 HAL_ERROR。 */
      Uart2_DebugTraceTx(HAL_ERROR, Length);
      return;
    }
  }

  if (Uart2_IsTxHardwareReady(uart2, &sr_value) == 0U)
  {
    /* TXE/TE/UE 不满足时才执行完整外设恢复，避免 RX 噪声导致 TX 引脚反复悬空。 */
    (void)Uart2_RecoverPeripheral(UART2_RECOVER_REASON_PRECHECK);
    /* 完整恢复后不立即发送，避免 SA=00000000 或 TXE 未稳定时仍启动 HAL_UART_Transmit_IT。 */
    Uart2_DebugTraceSkip(UART2_TX_SKIP_REASON_TX_NOT_READY, Length);
    return;
  }

  /* 复制到 UART2 驱动内部 TX 缓冲，保证中断发送期间数据不被上层复写。 */
  Common_CopyData(pData, Uart2_TXBuf, Length);
  /* 标记 UART2 TX 忙，直到发送完成回调清零。 */
  Uart2_TxBusy = 1U;
  /* 记录中断发送启动 tick，用于下一次发送判断是否超时。 */
  Uart2_TxStartTick = now_tick;
  /* 启动 UART2 中断发送；函数本身不等待整帧发送完成。 */
  status = HAL_UART_Transmit_IT(uart2, Uart2_TXBuf, Length);
  if (status != HAL_OK)
  {
    /* 启动失败时立刻清忙标志，避免后续心跳被永久挡住。 */
    Uart2_TxBusy = 0U;
    /* 启动失败后恢复 USART2，避免 HAL_BUSY/HAL_ERROR 状态继续累积。 */
    (void)Uart2_RecoverPeripheral(UART2_RECOVER_REASON_START_FAIL);
  }

  /* 通过 UART10 输出本次中断发送启动结果。 */
  Uart2_DebugTraceTx(status, Length);
#else
  /* status 保存 HAL_UART_Transmit 的返回值，便于判断 OK/BUSY/TIMEOUT。 */
  HAL_StatusTypeDef status;

  /* 发送 UART2 外部通信帧，并保留 HAL 层真实返回值。 */
  status = Bsp_UartTransmit(BSP_UART_PORT_2, pData, Length, 100);
  /* 通过 UART10 输出本次发送结果，现场串口助手可直接观察 ST 字段。 */
  Uart2_DebugTraceTx(status, Length);
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == Bsp_UartHandle(BSP_UART_PORT_2))
  {
    /* USART2 中断发送完成后清忙标志，下一帧心跳即可继续发送。 */
    Uart2_TxBusy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == Bsp_UartHandle(BSP_UART_PORT_2))
  {
    /* UART2 错误中断主要来自 RX 侧 FE/NE/ORE/LBD，不在这里清 TX 忙标志。 */
    /* 保留 TX 忙标志可以避免一包中断发送尚未结束时，被 RX 错误误判为空闲。 */
    /* 如果 TX 完成回调真的丢失，任务上下文会通过 20ms 超时走完整恢复。 */
    /* 只在中断上下文做轻量标记，RX DMA 重启留给任务上下文执行。 */
    Uart2_NeedRecover = 1U;
    /* 立即执行 SR/DR 清标志序列，尽快释放 FE/NE/ORE/IDLE。 */
    Uart2_ClearRxFaultFlags(huart);
  }
}

uint16_t Uart2_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

  //------------------------------------------------------------------
  Uart2_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_2);

  if (RemainLen != Uart2_Flag_Last)
  {
    Uart2_RecvWaitTimeCnt = 0;
    Uart2_Flag_Last = RemainLen;
  }
  else
  {
    if (Uart2_RecvWaitTimeCnt >= UART2_TimeoutComp)
    {
      if (RemainLen < UART2_MAX_PACKET_SIZE)
      {
        rlen = (UART2_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart2_DMABuf, data, rlen);

        Uart2_DMAReset();
      }

      Uart2_RecvWaitTimeCnt = 0;
	  }
  }

  return rlen;
}

void Uart2_DeInit(void)
{
  Bsp_UartDeInit(BSP_UART_PORT_2);
}
