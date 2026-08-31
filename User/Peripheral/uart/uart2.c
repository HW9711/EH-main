//uart2.c

#include "main.h"
#include "uart2.h"
#include "bsp_uart.h"
#include "board.h"
#include "common.h"
#include "diagnostic_config.h"
//#include "delay.h"

#include <string.h>
#include <stdio.h>

#define	UART2_TimeoutComp   3

static uint8_t Uart2_Flag_Last = 0;
static uint16_t Uart2_RecvWaitTimeCnt = 0;
static uint8_t Uart2_DMABuf[UART2_MAX_PACKET_SIZE] = { 0 };
static volatile uint8_t Uart2_RxRestartPending = 0U; /* UART2 错误回调只置位，实际 DMA 恢复放到30ms诊断任务，避免在中断中执行缓存清理。 */

volatile uint32_t g_uart2_rx_error_count = 0U;          /* 统计 USART2 接收错误，便于判断下行链是否被噪声中止。 */
volatile uint32_t g_uart2_rx_last_error = 0U;           /* 保存 HAL_UART_ERROR_* 位，暂停调试器后仍可追溯。 */
volatile uint32_t g_uart2_rx_dma_restart_count = 0U;    /* 只统计成功恢复次数，失败单独计数。 */
volatile uint32_t g_uart2_rx_dma_start_fail_count = 0U; /* 初始 DMA 启动与错误恢复共用此失败计数。 */
volatile uint32_t g_uart2_rx_packet_count = 0U;         /* 统计真正交给上层解析的数据包，不把空闲轮询计入。 */
volatile uint16_t g_uart2_rx_last_packet_length = 0U;   /* 保存最近命令包长度，DUMP+CRLF 正常应为12。 */

/*
 * 函数功能：启动 UART2 RX DMA，并在启动失败时留下诊断计数和待恢复标记。
 * 输入参数：无。
 * 返回参数：无；失败后由30ms诊断任务再次尝试恢复。
 */
static void Uart2_DmaInit(void)
{
//	Delay_ms(300);

  if (Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE) != HAL_OK)
  {
    ++g_uart2_rx_dma_start_fail_count; /* HAL 拒绝启动时不能伪装成接收已就绪。 */
    Uart2_RxRestartPending = 1U; /* 留给诊断任务重试，初始化阶段不阻塞其它业务启动。 */
  }
}

/*
 * 函数功能：按指定波特率重新初始化 UART2，供临时诊断模式切换到 115200。
 * 输入参数：baud 为目标波特率。
 * 返回参数：无；底层初始化失败时进入统一 Error_Handler。
 */
void Uart2_Configuration(uint32_t baud)
{
  /* UART2 初始化失败时进入统一故障处理，避免外控通信口处于不可预期状态。 */
  if (Bsp_UartInit(BSP_UART_PORT_2, baud) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 函数功能：停止并清空 UART2 RX DMA 后重新启动，用于取包完成和诊断接收错误恢复。
 * 输入参数：无。
 * 返回参数：无；恢复失败时保留待恢复标记并累计失败次数。
 */
static void Uart2_DMAReset(void)
{
  Bsp_UartDmaStop(BSP_UART_PORT_2);                            /* 取走一包数据后先停止 USART2 RX DMA，防止旧 DMA 状态继续写入缓存。 */
  memset(Uart2_DMABuf, 0, UART2_MAX_PACKET_SIZE);              /* 清空接收缓存，避免上一包残留影响下一次解析。 */
  if (Bsp_UartReceiveDma(BSP_UART_PORT_2, Uart2_DMABuf, UART2_MAX_PACKET_SIZE) == HAL_OK)
  {
    ++g_uart2_rx_dma_restart_count;                            /* HAL 确认启动成功后才记录一次有效恢复。 */
  }
  else
  {
    ++g_uart2_rx_dma_start_fail_count;                         /* 保存失败次数，调试器可区分线路无数据和 DMA 未运行。 */
    Uart2_RxRestartPending = 1U;                               /* 下一次诊断周期继续恢复，不在当前周期循环阻塞。 */
  }
  Uart2_RecvWaitTimeCnt = 0;                                   /* DMA 重启后重新累计静默时间。 */
  Uart2_Flag_Last = UART2_MAX_PACKET_SIZE;                     /* 复位剩余长度快照，让下一包从空缓存状态开始判断。 */
}

/*
 * 函数功能：初始化外部通信 UART2 接收，并确保 V4.0 RS485 芯片处于接收状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Uart2_Init(void)
{
  Uart2_RxRestartPending = 0U; /* 本次上电尚未发生接收错误，先清除上次软复位遗留的恢复请求。 */
  g_uart2_rx_error_count = 0U; /* 调试计数以本次上电为边界。 */
  g_uart2_rx_last_error = 0U; /* 尚未发生 HAL UART 错误。 */
  g_uart2_rx_dma_restart_count = 0U; /* 初始启动不计入错误恢复次数。 */
  g_uart2_rx_dma_start_fail_count = 0U; /* 尚未尝试启动 RX DMA。 */
  g_uart2_rx_packet_count = 0U; /* 尚未向上层交付任何数据包。 */
  g_uart2_rx_last_packet_length = 0U; /* 尚无可观察的命令长度。 */
#if (MOTOR_FOOT_TRACE_ENABLE == 1U) && (MOTOR_FOOT_TRACE_UART2_EXCLUSIVE == 1U)
  Uart2_Configuration(MOTOR_FOOT_TRACE_UART2_BAUD); /* 诊断固件在DMA启动前把UART2从正式9600切到115200，缩短故障后CSV阻塞时间。 */
#endif
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 低电平关闭驱动并使能接收，避免初始化阶段占用 RS485 总线。 */
  Uart2_DmaInit(); /* 方向稳定为接收后再启动 DMA，保证外部设备下行帧可以立即进入缓存。 */
}

/*
 * 函数功能：切换 V4.0 RS485 芯片到发送状态，阻塞发送完整帧后恢复接收状态。
 * 输入参数：pData 为待发送数据缓冲区；Length 为本次发送的字节数。
 * 返回参数：无。
 */
void Uart2_SendPacket(uint8_t *pData, uint16_t Length)
{
  (void)Uart2_SendPacketChecked(pData, Length, 100U); /* 正式旧调用保持原100ms上限，新增接口只补充返回值而不改变协议行为。 */
}

/*
 * 函数功能：切换 V4.0 RS485 芯片到发送状态，按指定超时发送后恢复接收并返回 HAL 结果。
 * 输入参数：data 为发送缓存；length 为字节数；timeout_ms 为单次阻塞上限。
 * 返回参数：HAL_OK 表示完整发送，其它 HAL 状态表示失败。
 */
HAL_StatusTypeDef Uart2_SendPacketChecked(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status; /* 保存底层 UART 发送结果，诊断任务只计数不循环重试。 */

  if((data == NULL) || (length == 0U))
  {
    return HAL_ERROR; /* 空缓存或零长度不得切换 RS485 方向。 */
  }

  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_SET); /* 高电平使能 DE 并关闭 /RE，避免发送数据被本机 DMA 回收。 */
  status = Bsp_UartTransmit(BSP_UART_PORT_2, (uint8_t *)data, length, timeout_ms); /* HAL API 未声明 const，只在接口边界去除限定且不修改调用方数据。 */
  HAL_GPIO_WritePin(BOARD_UART2_RS485_DIR_PORT, BOARD_UART2_RS485_DIR_PIN, GPIO_PIN_RESET); /* 成功、超时或错误都立即恢复接收，不能长期占用总线。 */
  return status; /* 调用方决定只计数或执行原静默策略。 */
}

/*
 * 函数功能：轮询 UART2 RX DMA，优先恢复诊断接收错误，并在完整换行或帧间静默后向上层交付一包数据。
 * 输入参数：data 为上层提供的接收缓存，容量不得小于 UART2_MAX_PACKET_SIZE。
 * 返回参数：实际交付字节数；0 表示本周期无完整包或正在恢复 DMA。
 */
uint16_t Uart2_DMARecvDataPeek(uint8_t *data)
{
  uint32_t RemainLen = 0;
  uint16_t rlen = 0;

#if (MOTOR_FOOT_TRACE_ENABLE == 1U) && (MOTOR_FOOT_TRACE_UART2_EXCLUSIVE == 1U)
  if (Uart2_RxRestartPending != 0U)
  {
    Uart2_RxRestartPending = 0U; /* 先消费本次请求；若 HAL 仍失败，复位函数会重新置位供下周期重试。 */
    Uart2_DMAReset(); /* 错误回调已完成 DMA 中止后再由任务恢复，避免心跳发送后只剩单向通信。 */
    return 0U; /* 恢复周期不解析旧缓存，防止把错误发生前的残片当成人工 DUMP 命令。 */
  }
#endif

  //------------------------------------------------------------------
  Uart2_RecvWaitTimeCnt++;
  RemainLen = Bsp_UartRxDmaRemain(BSP_UART_PORT_2);

#if (MOTOR_FOOT_TRACE_ENABLE == 1U) && (MOTOR_FOOT_TRACE_UART2_EXCLUSIVE == 1U)
  if (RemainLen < UART2_MAX_PACKET_SIZE)
  {
    uint16_t diagnostic_len = (uint16_t)(UART2_MAX_PACKET_SIZE - RemainLen); /* 诊断命令固定以换行结束，可脱离正式协议的三周期静默判定。 */
    if (Uart2_DMABuf[diagnostic_len - 1U] == (uint8_t)'\n')
    {
      Common_CopyData(Uart2_DMABuf, data, diagnostic_len); /* 收到完整 ARM/DUMP 文本后立即复制，按钮到冻结最多只等待一个30ms任务周期。 */
      ++g_uart2_rx_packet_count; /* 完整换行命令已经交给诊断解析器，证明电脑到主控的下行链路有效。 */
      g_uart2_rx_last_packet_length = diagnostic_len; /* 正常 EHDBG_DUMP+CRLF 应记录12，便于现场断点核对。 */
      Uart2_DMAReset(); /* 命令取走后立刻恢复DMA接收，主控回ACK期间仍保持半双工方向受控。 */
      return diagnostic_len;
    }
  }
#endif

  /* DMA 剩余数发生变化说明本帧仍在接收，重新开始帧间静默计时。 */
  if (RemainLen != Uart2_Flag_Last)
  {
    Uart2_RecvWaitTimeCnt = 0;
    Uart2_Flag_Last = RemainLen;
  }
  else
  {
    /* 接收长度连续不变达到门限后，才把缓存交给外控协议层。 */
    if (Uart2_RecvWaitTimeCnt >= UART2_TimeoutComp)
    {
      /* DMA 至少接收一个字节时才复制，空帧不触发上层解析。 */
      if (RemainLen < UART2_MAX_PACKET_SIZE)
      {
        rlen = (UART2_MAX_PACKET_SIZE - RemainLen);

        Common_CopyData(Uart2_DMABuf, data, rlen);
        ++g_uart2_rx_packet_count; /* 正式静默成包路径同样记录已交付包数量。 */
        g_uart2_rx_last_packet_length = rlen; /* 保存最近一次实际包长，不把DMA剩余长度误当成接收长度。 */

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

/*
 * 函数功能：接收 HAL 串口错误通知，并为诊断模式登记 UART2 RX DMA 的任务级恢复请求。
 * 输入参数：huart 为发生错误的 HAL 串口句柄。
 * 返回参数：无；非 UART2 错误保持原有空回调行为。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return; /* 本次临时诊断只处理外部通信 UART2，不能改变其它驱动串口的错误策略。 */
  }

  ++g_uart2_rx_error_count; /* 每次 HAL 通知只累计一次，具体错误位保存在下一行。 */
  g_uart2_rx_last_error = huart->ErrorCode; /* 保存 PE/NE/FE/ORE 位，定位接收脚高阻或线路噪声。 */
#if (MOTOR_FOOT_TRACE_ENABLE == 1U) && (MOTOR_FOOT_TRACE_UART2_EXCLUSIVE == 1U)
  Uart2_RxRestartPending = 1U; /* HAL 在 DMA 模式遇到任意接收错误会中止传输，交由30ms任务安全重启。 */
#endif
}
