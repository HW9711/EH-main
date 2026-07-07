#include "soft_uart.h"

#include "FreeRTOS.h"
#include "kernel_scheduler.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include "bsp_uart.h"
#include "Pubinterface.h"
#include "sscBEEP.h"

#include <stdbool.h>

/* 接收侧缓冲容量设计说明：
 * 1. 中断环形缓冲只承担“短时削峰”，避免 ISR 内直接访问队列导致开销过大。
 * 2. 任务队列容量更大，用于给上层消费者留出更宽松的读取窗口。 */
#define SOFT_UART_ISR_BUFFER_SIZE      64U
#define SOFT_UART_QUEUE_DEPTH          128U
#define SOFT_UART_DEFAULT_BAUDRATE     9600U
#define SOFT_UART_TIMER_INSTANCE       TIM11
#define SOFT_UART_TIMER_IRQn           TIM1_TRG_COM_TIM11_IRQn
#define SOFT_UART_IRQ_PRIORITY         4U  /* 软串口位采样必须高于 FreeRTOS 临界区屏蔽阈值 5，避免 TIM11 采样被任务临界区延迟导致错帧。 */
#define SOFT_UART_DIAG_LED             0

#define CS1237_FRAME_LENGTH            21U
#define CS1237_HEADER_0                0xAAU
#define CS1237_HEADER_1                0x55U
#define CS1237_TAIL_0                  0x55U
#define CS1237_TAIL_1                  0xAAU
#define CS1237_PROTOCOL_VER            0x02U
#define CS1237_MSG_TYPE_REPORT         0x01U
#define CS1237_PAYLOAD_LENGTH          0x0CU
#define CS1237_CRC_OFFSET              17U
#define CS1237_CRC_LENGTH              15U
/* CS1237 下位机设备码映射业务泵类型；未列出的编码先作为备用码处理，不参与泵类型识别。 */
#define CS1237_DEVICE_CODE_INJECT_WATER 0x00U  /* 0x00 表示注水泵，PUMPA/PUMPB 按注水方向和流量公式输出。 */
#define CS1237_DEVICE_CODE_POUR_WATER   0x08U  /* 0x08 表示灌注泵，PUMPA/PUMPB 按灌注方向和流量公式输出。 */
#define CS1237_DEVICE_CODE_DRAW_WATER   0x09U  /* 0x09 表示抽水泵，PUMPA/PUMPB 按抽水方向和流量公式输出。 */
#define CS1237_PUMP_LOSS_SUSPECT_MS     1500U  /* 超过 1.5 秒无有效帧先进入疑似丢失，避免单次软串口错帧立刻清在线状态。 */
#define CS1237_PUMP_LOSS_CONFIRM_MS     3000U  /* 疑似丢失持续到 3 秒仍无有效帧才确认离线，兼顾拔泵响应和偶发错帧容错。 */

/* 位级接收状态机阶段定义。
 * 采用“起始位确认 -> 8 位数据 -> 停止位确认”的 8N1 接收流程。 */
typedef enum {
    SOFT_UART_RX_STAGE_IDLE = 0,
    SOFT_UART_RX_STAGE_START,
    SOFT_UART_RX_STAGE_DATA,
    SOFT_UART_RX_STAGE_STOP
} SoftUartRxStage;

/* 单通道上下文。
 * 每路都保存自身 GPIO 配置、消息队列、ISR 环形缓冲和诊断计数。 */
typedef struct {
    GPIO_TypeDef *tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef *rx_port;
    uint16_t rx_pin;
    uint32_t baudrate;
    uint16_t bit_time_us;
    uint16_t half_bit_time_us;

    QueueHandle_t queue_handle;
    StaticQueue_t queue_control;
    uint8_t queue_storage[SOFT_UART_QUEUE_DEPTH];

    volatile uint8_t ring_buffer[SOFT_UART_ISR_BUFFER_SIZE];
    volatile uint16_t ring_head;
    volatile uint16_t ring_tail;
    volatile uint16_t ring_count;

    volatile uint32_t received_bytes;
    volatile uint32_t queue_overflow_count;
    volatile uint32_t buffer_overflow_count;
    volatile uint32_t overlap_drop_count;
    volatile uint32_t framing_error_count;

    volatile uint8_t exti_suppressed;
} SoftUartChannelContext;

/* 全局接收器状态。
 * 依据用户约束，两路不会正常同时上报，因此任意时刻只允许一个 active channel。 */
typedef struct {
    volatile sim_uart_channel_t active_channel;
    volatile SoftUartRxStage stage;
    volatile uint8_t bit_index;
    volatile uint8_t current_byte;
} SoftUartReceiverState;

/* 每路协议帧解析状态。
 * 任务层按字节寻找 AA55 帧头，拼满固定长度后再校验字段和 CRC。 */
typedef struct {
    uint8_t frame[CS1237_FRAME_LENGTH];
    uint8_t length;
} Cs1237FrameParser;

static SoftUartChannelContext s_channels[SIM_UART_COUNT] = {
    {
        .tx_port = GPIOE,
        .tx_pin = GPIO_PIN_5,
        .rx_port = GPIOE,
        .rx_pin = GPIO_PIN_4,
        .baudrate = SOFT_UART_DEFAULT_BAUDRATE,
    },
    {
        .tx_port = GPIOE,
        .tx_pin = GPIO_PIN_11,
        .rx_port = GPIOE,
        .rx_pin = GPIO_PIN_6,
        .baudrate = SOFT_UART_DEFAULT_BAUDRATE,
    }
};

static SoftUartReceiverState s_receiver = {
    .active_channel = SIM_UART_NONE,
    .stage = SOFT_UART_RX_STAGE_IDLE,
    .bit_index = 0U,
    .current_byte = 0U
};

static kernel_task_t sSimUartTaskHandle;
static uint8_t s_initialized = 0U;
static uint8_t s_task_created = 0U;
static uint8_t s_dwt_ready = 0U;
static uint8_t s_tim7_ready = 0U;
static uint8_t s_timer_initialized = 0U;
static Cs1237FrameParser s_cs1237_parsers[SIM_UART_COUNT];
static uint32_t s_cs1237_last_valid_tick[SIM_UART_COUNT]; /* 记录每路最近一次有效帧时间，用于拔泵后无数据场景的离线判定。 */
static uint32_t s_cs1237_loss_suspect_tick[SIM_UART_COUNT]; /* 记录每路首次进入疑似丢失的时间，用于连续确认后再清在线状态。 */

/* 私有函数声明区：
 * 这些函数按“时基/硬件控制 -> 缓冲区 -> 状态机 -> 对外接口”的顺序组织。 */
static void SoftUart_InitTimingBase(void);
static void SoftUart_InitRxTimer(void);
static void SoftUart_InitChannelGpio(sim_uart_channel_t channel);
static void SoftUart_StartSampleTimer(uint16_t delay_us);
static void SoftUart_StopSampleTimer(void);
static void SoftUart_EnableExti(sim_uart_channel_t channel);
static void SoftUart_DisableExti(sim_uart_channel_t channel);
static void SoftUart_RearmExtiLines(void);
static void SoftUart_RingPushFromIsr(sim_uart_channel_t channel, uint8_t data);
static bool SoftUart_RingPopTask(sim_uart_channel_t channel, uint8_t *data);
static void SoftUart_FinishReceive(bool byte_valid);
static void SoftUart_ProcessSample(void);
static void SimUartTaskFunc(uint32_t event);
static SoftUartChannelContext *SoftUart_GetChannel(sim_uart_channel_t channel);
static uint32_t SoftUart_GetTimerClockHz(void);
static void SoftUart_WriteTx(const SoftUartChannelContext *channel, GPIO_PinState level);
static GPIO_PinState SoftUart_ReadRx(const SoftUartChannelContext *channel);
static bool SoftUart_TestForwardChannelSelected(sim_uart_channel_t channel);
static void SoftUart_TestForwardFrame(sim_uart_channel_t channel, const uint8_t *data, uint16_t length);
static uint16_t Cs1237_CalcCrc16Modbus(const uint8_t *data, uint16_t length);
static uint16_t Cs1237_ReadU16Le(const uint8_t *data);
static uint32_t Cs1237_ReadU32Le(const uint8_t *data);
static bool Cs1237_FrameValid(const uint8_t *frame);
static pumpMessage_t *Cs1237_GetPumpMessageForChannel(sim_uart_channel_t channel);
static void Cs1237_BeepOnceIfNoAlarm(void);
static void Cs1237_RefreshPumpDisplayByChannel(sim_uart_channel_t channel);
static void Cs1237_UpdatePumpMessage(sim_uart_channel_t channel, const uint8_t *frame);
static void Cs1237_ServicePumpLossTimeout(void);
static void Cs1237_ParserResync(Cs1237FrameParser *parser);
static void Cs1237_ParseByte(sim_uart_channel_t channel, uint8_t data);
static void delay_us(uint32_t us);

static bool SoftUart_ChannelValid(sim_uart_channel_t channel)
{
    return (channel == SIM_UART_1) || (channel == SIM_UART_2);
}

/* 根据通道号获取上下文。
 * 所有对外接口都会先经过该函数做统一的通道合法性检查。 */
static SoftUartChannelContext *SoftUart_GetChannel(sim_uart_channel_t channel)
{
    if (!SoftUart_ChannelValid(channel))
    {
        return NULL;
    }

    return &s_channels[(uint32_t)channel];
}

/*
 * 函数功能：按模拟串口通道取得对应的 A/B 泵公共状态结构。
 * 输入参数：channel 为压力模块模拟串口通道，当前线束要求 SIM_UART_1/PE4 接 A 泵压力传感器，SIM_UART_2/PE6 接 B 泵压力传感器。
 * 返回参数：有效通道返回对应 pumpMessage_t 指针，非法通道返回 NULL。
 */
static pumpMessage_t *Cs1237_GetPumpMessageForChannel(sim_uart_channel_t channel)
{
    if (channel == SIM_UART_1)
    {
        return &pumpMessageA; /* PE4 收到的是 A 泵压力帧，只修正压力数据归属，不改变 A 泵驱动串口。 */
    }

    if (channel == SIM_UART_2)
    {
        return &pumpMessageB; /* PE6 收到的是 B 泵压力帧，只修正压力数据归属，不改变 B 泵驱动串口。 */
    }

    return NULL; /* 非法通道不能写泵状态，避免越界访问公共状态。 */
}

/*
 * 函数功能：在系统无全局报警时发送一次普通提示蜂鸣。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Cs1237_BeepOnceIfNoAlarm(void)
{
    if (WorkMessage.alarm_flag == false)
    {
        SendKeyBeepMessage(1U); /* 普通蜂鸣会清蜂鸣任务内部报警态，所以只在无报警时提示泵接入/丢失。 */
    }
}

/*
 * 函数功能：按压力模块通道刷新对应屏幕泵区域。
 * 输入参数：channel 为压力模块模拟串口通道。
 * 返回参数：无。
 */
static void Cs1237_RefreshPumpDisplayByChannel(sim_uart_channel_t channel)
{
    if (channel == SIM_UART_1)
    {
        Pubinterface_RefreshPumpADisplay(); /* PE4 对应 A 泵压力状态，在线/离线变化后只刷新左侧 A 泵显示。 */
    }
    else if (channel == SIM_UART_2)
    {
        Pubinterface_RefreshPumpBDisplay(); /* PE6 对应 B 泵压力状态，在线/离线变化后只刷新右侧 B 泵显示。 */
    }
}

static void SoftUart_WriteTx(const SoftUartChannelContext *channel, GPIO_PinState level)
{
    HAL_GPIO_WritePin(channel->tx_port, channel->tx_pin, level);
}

static GPIO_PinState SoftUart_ReadRx(const SoftUartChannelContext *channel)
{
    return HAL_GPIO_ReadPin(channel->rx_port, channel->rx_pin);
}

/* 判断当前字节是否需要做 UART10 测试透传。
 * 该配置默认关闭，只有显式选择某一路时才会额外输出。 */
static bool SoftUart_TestForwardChannelSelected(sim_uart_channel_t channel)
{
    switch (SOFT_UART_TEST_FORWARD_SOURCE)
    {
    case SOFT_UART_TEST_FORWARD_SIM_UART_1:
        return channel == SIM_UART_1;

    case SOFT_UART_TEST_FORWARD_SIM_UART_2:
        return channel == SIM_UART_2;

    case SOFT_UART_TEST_FORWARD_OFF:
    default:
        return false;
    }
}

/* 通过 UART10 透传当前批次收到的原始协议字节。
 * 这里不添加任何前缀、换行或格式化文本，便于串口助手直接观察真实帧内容。 */
static void SoftUart_TestForwardFrame(sim_uart_channel_t channel, const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    if (!SoftUart_TestForwardChannelSelected(channel))
    {
        return;
    }

    (void)Bsp_UartTransmit(BSP_UART_PORT_10,
                           (uint8_t *)data,
                           length,
                           SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS);
}

/* 初始化发送侧微秒延时基准。
 * 这里优先使用 DWT 计数器，若目标环境不可用，则退回到 TIM7 计数器。
 * 注意：TIM7 只用于发送延时，不参与 RX 采样，避免破坏现有 delay.c 的使用方式。 */
static void SoftUart_InitTimingBase(void)
{
    uint32_t start;

    if (s_dwt_ready || s_tim7_ready)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    start = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 1000U; i++)
    {
    }
    s_dwt_ready = (DWT->CYCCNT != start) ? 1U : 0U;

    if (!s_dwt_ready)
    {
        start = __HAL_TIM_GET_COUNTER(&htim7);
        for (volatile uint32_t i = 0U; i < 1000U; i++)
        {
        }
        s_tim7_ready = (__HAL_TIM_GET_COUNTER(&htim7) != start) ? 1U : 0U;
    }
}

/* 微秒延时函数仅用于 TX bit-bang。
 * RX 时序统一由 TIM11 中断驱动，避免阻塞式接收占用 CPU。 */
static void delay_us(uint32_t us)
{
    if (s_dwt_ready)
    {
        uint32_t start = DWT->CYCCNT;
        uint32_t cycles = us * (SystemCoreClock / 1000000UL);

        while ((DWT->CYCCNT - start) < cycles)
        {
        }
        return;
    }

    if (s_tim7_ready)
    {
        uint32_t remaining = us;
        uint32_t start = __HAL_TIM_GET_COUNTER(&htim7);

        while (remaining > 0U)
        {
            uint32_t now = __HAL_TIM_GET_COUNTER(&htim7);
            uint32_t elapsed = (now >= start) ? (now - start) : (0x10000U + now - start);
            if (elapsed >= remaining)
            {
                break;
            }
            remaining -= elapsed;
            start = now;
        }
        return;
    }

    if (us >= 1000U)
    {
        HAL_Delay(us / 1000U);
        us %= 1000U;
    }

    for (volatile uint32_t i = 0U; i < (us * (SystemCoreClock / 1000000U / 4U + 1U)); i++)
    {
        __NOP();
    }
}

static uint32_t SoftUart_GetTimerClockHz(void)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t pFLatency = 0U;
    uint32_t apb2_prescaler;
    uint32_t tim_clock;

    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
    apb2_prescaler = clkconfig.APB2CLKDivider;

    if (apb2_prescaler == RCC_HCLK_DIV1)
    {
        tim_clock = HAL_RCC_GetPCLK2Freq();
    }
    else
    {
        tim_clock = 2UL * HAL_RCC_GetPCLK2Freq();
    }

    return tim_clock;
}

/* 初始化专用 RX 采样定时器 TIM11。
 * 定时器预分频到 1MHz，便于直接使用“微秒”为单位装载 ARR。 */
static void SoftUart_InitRxTimer(void)
{
    uint32_t prescaler;

    if (s_timer_initialized)
    {
        return;
    }

    __HAL_RCC_TIM11_CLK_ENABLE();

    prescaler = (SoftUart_GetTimerClockHz() / 1000000U);
    if (prescaler == 0U)
    {
        prescaler = 1U;
    }

    SOFT_UART_TIMER_INSTANCE->CR1 = 0U;
    SOFT_UART_TIMER_INSTANCE->PSC = (uint16_t)(prescaler - 1U);
    SOFT_UART_TIMER_INSTANCE->ARR = 0xFFFFU;
    SOFT_UART_TIMER_INSTANCE->CNT = 0U;
    SOFT_UART_TIMER_INSTANCE->SR = 0U;
    SOFT_UART_TIMER_INSTANCE->DIER = 0U;
    SOFT_UART_TIMER_INSTANCE->EGR = TIM_EGR_UG;
    SOFT_UART_TIMER_INSTANCE->SR = 0U;

    HAL_NVIC_SetPriority(SOFT_UART_TIMER_IRQn, SOFT_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(SOFT_UART_TIMER_IRQn);

    s_timer_initialized = 1U;
}

/* 启动一次单次采样。
 * 第一次采样使用半位时间对齐，之后每次按 1bit 周期触发。 */
static void SoftUart_StartSampleTimer(uint16_t delay_us)
{
    if (delay_us == 0U)
    {
        delay_us = 1U;
    }

    SOFT_UART_TIMER_INSTANCE->CR1 = 0U;
    SOFT_UART_TIMER_INSTANCE->CNT = 0U;
    SOFT_UART_TIMER_INSTANCE->ARR = (uint32_t)delay_us - 1U;
    SOFT_UART_TIMER_INSTANCE->EGR = TIM_EGR_UG;
    SOFT_UART_TIMER_INSTANCE->SR = 0U;
    SOFT_UART_TIMER_INSTANCE->DIER = TIM_DIER_UIE;
    SOFT_UART_TIMER_INSTANCE->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
}

/* 停止采样定时器，结束当前字节接收。 */
static void SoftUart_StopSampleTimer(void)
{
    SOFT_UART_TIMER_INSTANCE->CR1 &= ~TIM_CR1_CEN;
    SOFT_UART_TIMER_INSTANCE->DIER = 0U;
    SOFT_UART_TIMER_INSTANCE->SR = 0U;
}

static void SoftUart_EnableExti(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if (ctx == NULL)
    {
        return;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(ctx->rx_pin);
    EXTI->IMR |= ctx->rx_pin;
    ctx->exti_suppressed = 0U;
}

/* 忙期间关闭指定通道 EXTI，防止重复进入或跨通道抢占。 */
static void SoftUart_DisableExti(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if (ctx == NULL)
    {
        return;
    }

    EXTI->IMR &= ~(uint32_t)ctx->rx_pin;
    __HAL_GPIO_EXTI_CLEAR_IT(ctx->rx_pin);
}

/* 一个字节接收结束后重新开放两路线的起始位捕获。 */
static void SoftUart_RearmExtiLines(void)
{
    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        SoftUart_EnableExti((sim_uart_channel_t)i);
    }
}

/* 初始化单路 GPIO。
 * TX 为推挽输出并默认拉高空闲，RX 采用下降沿 EXTI 检测起始位。 */
static void SoftUart_InitChannelGpio(sim_uart_channel_t channel)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if (ctx == NULL)
    {
        return;
    }

    GPIO_InitStruct.Pin = ctx->tx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ctx->tx_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ctx->rx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ctx->rx_port, &GPIO_InitStruct);

    SoftUart_WriteTx(ctx, GPIO_PIN_SET);
    __HAL_GPIO_EXTI_CLEAR_IT(ctx->rx_pin);
}

/* 在中断环境中向环形缓冲压入 1 字节。
 * 若缓冲已满，只累加溢出计数，不在 ISR 中做阻塞处理。 */
static void SoftUart_RingPushFromIsr(sim_uart_channel_t channel, uint8_t data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if (ctx == NULL)
    {
        return;
    }

    if (ctx->ring_count >= SOFT_UART_ISR_BUFFER_SIZE)
    {
        ctx->buffer_overflow_count++;
        return;
    }

    ctx->ring_buffer[ctx->ring_head] = data;
    ctx->ring_head = (uint16_t)((ctx->ring_head + 1U) % SOFT_UART_ISR_BUFFER_SIZE);
    ctx->ring_count++;
    ctx->received_bytes++;
}

/*
 * 函数功能：在任务上下文从 ISR 环形缓冲取出 1 字节。
 * 输入参数：channel 为模拟串口通道；data 为输出字节指针。
 * 返回参数：成功取到字节返回 true，无数据或参数无效返回 false。
 */
static bool SoftUart_RingPopTask(sim_uart_channel_t channel, uint8_t *data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);
    bool has_data = false;
    uint32_t primask; /* 保存进入临界区前的全局中断状态，退出时按原状态恢复，避免破坏外层关中断状态。 */

    if ((ctx == NULL) || (data == NULL))
    {
        return false;
    }

    primask = __get_PRIMASK(); /* 软串口 ISR 优先级高于 FreeRTOS 屏蔽阈值，BASEPRI 不能保护下面的环形缓冲共享索引。 */
    __disable_irq(); /* 暂停所有中断，防止 TIM11 ISR 正在写 ring_count/head 时任务同时修改 tail/count。 */
    if (ctx->ring_count > 0U)
    {
        *data = ctx->ring_buffer[ctx->ring_tail];
        ctx->ring_tail = (uint16_t)((ctx->ring_tail + 1U) % SOFT_UART_ISR_BUFFER_SIZE);
        ctx->ring_count--;
        has_data = true;
    }
    if (primask == 0U)
    {
        __enable_irq(); /* 只有进入前允许中断时才重新打开，避免破坏调用方已有的全局关中断保护。 */
    }

    return has_data;
}

/* 结束本次字节接收流程。
 * 若停止位合法，则把组好的字节写入对应通道的 ISR 缓冲。 */
static void SoftUart_FinishReceive(bool byte_valid)
{
    sim_uart_channel_t channel = s_receiver.active_channel;

    if (SoftUart_ChannelValid(channel) && byte_valid)
    {
        SoftUart_RingPushFromIsr(channel, s_receiver.current_byte);
    }

    SoftUart_StopSampleTimer();
    s_receiver.active_channel = SIM_UART_NONE;
    s_receiver.stage = SOFT_UART_RX_STAGE_IDLE;
    s_receiver.bit_index = 0U;
    s_receiver.current_byte = 0U;

    SoftUart_RearmExtiLines();
}

/* 位级采样状态机核心。
 * 该函数由 TIM11 中断驱动，每次只处理一个采样点，直到完成 1 字节。 */
static void SoftUart_ProcessSample(void)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(s_receiver.active_channel);
    GPIO_PinState pin_state;

    if (ctx == NULL)
    {
        SoftUart_FinishReceive(false);
        return;
    }

    pin_state = SoftUart_ReadRx(ctx);

    switch (s_receiver.stage)
    {
        case SOFT_UART_RX_STAGE_START:
            if (pin_state != GPIO_PIN_RESET)
            {
                ctx->framing_error_count++;
                SoftUart_FinishReceive(false);
                return;
            }

            s_receiver.stage = SOFT_UART_RX_STAGE_DATA;
            s_receiver.bit_index = 0U;
            s_receiver.current_byte = 0U;
            SoftUart_StartSampleTimer(ctx->bit_time_us);
            break;

        case SOFT_UART_RX_STAGE_DATA:
            if (pin_state != GPIO_PIN_RESET)
            {
                s_receiver.current_byte |= (uint8_t)(1U << s_receiver.bit_index);
            }

            s_receiver.bit_index++;
            if (s_receiver.bit_index >= 8U)
            {
                s_receiver.stage = SOFT_UART_RX_STAGE_STOP;
            }

            SoftUart_StartSampleTimer(ctx->bit_time_us);
            break;

        case SOFT_UART_RX_STAGE_STOP:
            if (pin_state == GPIO_PIN_SET)
            {
                SoftUart_FinishReceive(true);
            }
            else
            {
                ctx->framing_error_count++;
                SoftUart_FinishReceive(false);
            }
            break;

        case SOFT_UART_RX_STAGE_IDLE:
        default:
            SoftUart_FinishReceive(false);
            break;
    }
}

static uint16_t Cs1237_CalcCrc16Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t Cs1237_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U);
}

static uint32_t Cs1237_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/*
 * 函数功能：检查 21 字节 CS1237 压力帧的固定字段和 CRC 是否有效。
 * 输入参数：frame 指向待检查的 21 字节候选帧。
 * 返回参数：帧头、固定字段、帧尾和 CRC 全部通过时返回 true，否则返回 false。
 */
static bool Cs1237_FrameValid(const uint8_t *frame)
{
    uint16_t frame_crc;
    uint16_t calc_crc;

    if ((frame[0] != CS1237_HEADER_0) || (frame[1] != CS1237_HEADER_1))
    {
        return false; /* 帧头不匹配时不能作为压力帧处理，避免错位数据刷新泵在线状态。 */
    }

    if ((frame[2] != CS1237_PROTOCOL_VER) ||
        (frame[3] != CS1237_MSG_TYPE_REPORT) ||
        (frame[4] != CS1237_PAYLOAD_LENGTH))
    {
        return false; /* 固定协议字段不符合当前压力上报格式，不能写入业务泵状态。 */
    }

    if ((frame[19] != CS1237_TAIL_0) || (frame[20] != CS1237_TAIL_1))
    {
        return false; /* 帧尾不匹配时说明候选窗口不完整，保持上一帧有效在线状态。 */
    }

    frame_crc = Cs1237_ReadU16Le(&frame[CS1237_CRC_OFFSET]);
    calc_crc = Cs1237_CalcCrc16Modbus(&frame[2], CS1237_CRC_LENGTH);

    return frame_crc == calc_crc; /* CRC 通过才允许刷新泵数据，避免单字节错误污染在线和压力状态。 */
}

/*
 * 函数功能：把 CS1237 模拟串口帧中的设备码转换成业务泵类型。
 * 输入参数：device_code 为下位机上报帧第 16 字节设备码。
 * 返回参数：DRAWWATER/INJECTWATER/POURWATER 表示已识别泵类型，0 表示备用码或未知码。
 */
static uint16_t Cs1237_MapDeviceCodeToPumpType(uint8_t device_code)
{
    /* 设备码只在这里转换为业务类型，避免外部通信或手柄联动路径再固定覆盖泵类型。 */
    switch (device_code)
    {
        case CS1237_DEVICE_CODE_INJECT_WATER:
            return INJECTWATER; /* 0x00 明确识别为注水泵，允许手柄冷却联动。 */
        case CS1237_DEVICE_CODE_POUR_WATER:
            return POURWATER; /* 0x08 明确识别为灌注泵，不参与手柄冷却联动。 */
        case CS1237_DEVICE_CODE_DRAW_WATER:
            return DRAWWATER; /* 0x09 明确识别为抽水泵，不参与手柄冷却联动。 */
        default:
            return 0U; /* 其它设备码为备用码，当前不强行映射为任何已知泵类型。 */
    }
}

/*
 * 函数功能：把一帧有效 CS1237 模拟串口数据同步到对应 A/B 泵运行状态。
 * 输入参数：channel 表示模拟串口通道，frame 指向已通过帧头、长度和 CRC 校验的 21 字节帧。
 * 返回参数：无。
 */
static void Cs1237_UpdatePumpMessage(sim_uart_channel_t channel, const uint8_t *frame)
{
    pumpMessage_t *pump_message;
    int32_t raw_cs1237 = (int32_t)Cs1237_ReadU32Le(&frame[6]);
    uint32_t weight_x10 = Cs1237_ReadU32Le(&frame[10]);
    uint16_t threshold_g = Cs1237_ReadU16Le(&frame[14]);
    uint8_t device_code = frame[16];
    uint16_t pump_type = Cs1237_MapDeviceCodeToPumpType(device_code);
    bool new_online_flag = (pump_type != 0U);
    uint16_t old_pump_type;
    bool old_online_flag;
    bool pump_online_changed;
    bool pump_display_changed;
    uint32_t now_tick = HAL_GetTick();

    pump_message = Cs1237_GetPumpMessageForChannel(channel);
    if (pump_message == NULL)
    {
        return; /* 非法通道不能写泵状态，也不能刷新屏幕，保持原有运行状态不变。 */
    }

    s_cs1237_last_valid_tick[(uint32_t)channel] = now_tick; /* 有效帧到达即刷新保活时间，供拔泵无数据超时判定使用。 */
    s_cs1237_loss_suspect_tick[(uint32_t)channel] = 0U; /* 有效帧恢复说明通信链路重新可信，清掉之前的疑似丢失等待状态。 */

    taskENTER_CRITICAL();
    old_pump_type = pump_message->type; /* 记录本帧前的业务泵类型，只在识别变化时刷新屏幕，避免每帧压满 UIDP 队列。 */
    old_online_flag = pump_message->online_flag; /* 记录本帧前在线状态，未识别/重新识别时需要让屏幕可用状态同步变化。 */
    pump_message->pressure_value = raw_cs1237; /* 保存压力原始值，泵任务后续按该值做压力堵塞保护。 */
    pump_message->weight_x10 = weight_x10; /* 保存 0.1g 重量值，供屏幕或外部通信读取压力模块当前重量。 */
    pump_message->pressure_threshold = threshold_g; /* 保存压力模块阈值，供压力报警弹窗和堵塞逻辑使用。 */
    /* 把设备码转换后的业务泵类型写入公共状态，后续泵任务按该类型选择方向和换算公式。 */
    pump_message->type = pump_type;
    pump_message->seq = frame[5]; /* 保存下位机帧序号，便于后续诊断压力模块是否连续上报。 */
    pump_message->online_flag = new_online_flag; /* 只有设备码映射到业务泵类型时才认为泵在线，备用码不允许启动泵。 */
    if (new_online_flag != false)
    {
        pump_message->losses_times = 0U; /* 识别恢复后清丢失计数，表示当前泵类型已重新可信。 */
    }
    else
    {
        if (pump_message->losses_times < 0xFFU)
        {
            pump_message->losses_times++; /* 未知设备码按识别丢失累计，饱和保护避免长时间运行后溢出回零。 */
        }
        pump_message->run_flag = false; /* 泵类型已经无效，强制停泵，避免沿用上一帧在线时的运行请求。 */
        pump_message->timingDrainage_flag = false; /* 清定时排空请求，避免重新识别后继承未知码阶段的排空状态。 */
        pump_message->speed_output = 0U; /* 实际输出速度清零，屏幕和外部通信都不能继续显示旧输出。 */
        pump_message->pressure_hold_flag = false; /* 泵已不可信时清压力锁存，下一次有效识别重新建立压力保护状态。 */
        pump_message->pressure_recover_ms = 0U; /* 清压力恢复计数，避免未知码期间残留旧压力恢复阶段。 */
    }
    pump_online_changed = (old_online_flag != new_online_flag); /* 只在在线/离线边沿蜂鸣，避免每帧重复响。 */
    pump_display_changed = ((old_pump_type != pump_type) ||
                            pump_online_changed); /* 类型或在线状态变化才触发 A/B 对应区域重绘，保持 A 左 B 右不换位。 */
    taskEXIT_CRITICAL();

    if (pump_display_changed)
    {
        if(pump_message->type==INJECTWATER)
        {
        pump_message->speed_work = Pubinterface_GetInjectionPumpStartFlow(); /* 注水泵上线时先按当前手柄 Page4 默认流量装载，未选中手柄或非法值则回退 30。 */
        }
        else if(pump_message->type==POURWATER)
        {
        pump_message->speed_work=200;
        }
        else if(pump_message->type==DRAWWATER)
        {
        pump_message->speed_work=10;
        }
        Cs1237_RefreshPumpDisplayByChannel(channel); /* 泵类型或在线状态变化后立即刷新对应泵区，保证屏幕可用态同步。 */
    }

    if (pump_online_changed)
    {
        Cs1237_BeepOnceIfNoAlarm(); /* 泵识别接入或识别丢失只在状态边沿蜂鸣一次，避免连续帧重复提示。 */
    }
}

/*
 * 函数功能：周期检查 CS1237 有效帧超时，先标记疑似丢失，连续确认后再处理拔泵离线状态。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Cs1237_ServicePumpLossTimeout(void)
{
    uint32_t now_tick = HAL_GetTick();

    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        sim_uart_channel_t channel = (sim_uart_channel_t)i;
        pumpMessage_t *pump_message = Cs1237_GetPumpMessageForChannel(channel);
        bool loss_confirmed = false;
        uint32_t loss_age_ms; /* 当前距离最近一次有效压力帧的时间，用于区分正常间隔、疑似丢失和确认离线。 */
        uint32_t suspect_age_ms; /* 当前疑似丢失已经持续的时间，用于避免一次错帧就清在线状态。 */

        if (pump_message == NULL)
        {
            continue; /* 非法通道没有业务泵状态，不能参与超时清理。 */
        }

        if (pump_message->online_flag == false)
        {
            s_cs1237_loss_suspect_tick[i] = 0U; /* 已经离线时没有疑似等待意义，清状态避免下次接入继承旧时间。 */
            continue; /* 已经离线的泵不重复蜂鸣，避免拔泵后每个周期都提示。 */
        }

        loss_age_ms = (uint32_t)(now_tick - s_cs1237_last_valid_tick[i]); /* 用无符号差值兼容 HAL tick 回绕。 */
        if (loss_age_ms <= CS1237_PUMP_LOSS_SUSPECT_MS)
        {
            s_cs1237_loss_suspect_tick[i] = 0U; /* 有效帧间隔仍在正常窗口内，清掉可能存在的疑似丢失状态。 */
            continue; /* 最近仍收到有效帧，保持当前在线状态和运行状态不变。 */
        }

        if (s_cs1237_loss_suspect_tick[i] == 0U)
        {
            s_cs1237_loss_suspect_tick[i] = now_tick; /* 首次超过 1.5 秒只记录疑似时间，不立刻清在线状态。 */
        }

        suspect_age_ms = (uint32_t)(now_tick - s_cs1237_loss_suspect_tick[i]); /* 计算疑似丢失连续持续了多久。 */
        if ((loss_age_ms <= CS1237_PUMP_LOSS_CONFIRM_MS) &&
            (suspect_age_ms <= (CS1237_PUMP_LOSS_CONFIRM_MS - CS1237_PUMP_LOSS_SUSPECT_MS)))
        {
            continue; /* 尚未达到确认离线窗口，继续保持在线状态，避免屏幕周期性闪离线。 */
        }

        taskENTER_CRITICAL();
        if ((pump_message->online_flag != false) &&
            ((uint32_t)(now_tick - s_cs1237_last_valid_tick[i]) > CS1237_PUMP_LOSS_CONFIRM_MS))
        {
            pump_message->online_flag = false; /* 超时确认泵丢失，屏幕和外部通信后续都读到离线。 */
            pump_message->type = 0U; /* 清业务泵类型，避免屏幕或控制路径沿用旧类型继续允许启动。 */
            pump_message->run_flag = false; /* 泵丢失时强制退出运行，避免后续泵任务继续按旧状态输出。 */
            pump_message->timingDrainage_flag = false; /* 清定时排空状态，避免重新接入后继承拔泵前的排空请求。 */
            pump_message->speed_output = 0U; /* 实际输出速度归零，运行态显示和外部通信都不能保留旧输出。 */
            pump_message->pressure_hold_flag = false; /* 泵已经离线，清压力锁存，下一次接入重新按新状态判断。 */
            pump_message->pressure_recover_ms = 0U; /* 清压力恢复计数，避免离线期间残留旧压力恢复阶段。 */
            if (pump_message->losses_times < 0xFFU)
            {
                pump_message->losses_times++; /* 记录一次超时丢失，饱和保护避免长时间运行后溢出回零。 */
            }
            s_cs1237_loss_suspect_tick[i] = 0U; /* 已经确认离线并清业务状态，疑似阶段结束，等待下一次有效接入重新开始。 */
            loss_confirmed = true; /* 标记本周期刚发生在线到离线边沿，后续只响一次并刷新 UI。 */
        }
        taskEXIT_CRITICAL();

        if (loss_confirmed != false)
        {
            Cs1237_RefreshPumpDisplayByChannel(channel); /* 离线状态已经写入公共结构，立即把对应泵区刷为不可用。 */
            Cs1237_BeepOnceIfNoAlarm(); /* 泵丢失边沿蜂鸣一次，报警期间不抢占报警蜂鸣。 */
        }
    }
}

static void Cs1237_ParserResync(Cs1237FrameParser *parser)
{
    uint8_t new_length = 0U;

    for (uint8_t start = 1U; (start + 1U) < parser->length; start++)
    {
        if ((parser->frame[start] == CS1237_HEADER_0) &&
            (parser->frame[start + 1U] == CS1237_HEADER_1))
        {
            new_length = (uint8_t)(parser->length - start);
            for (uint8_t i = 0U; i < new_length; i++)
            {
                parser->frame[i] = parser->frame[start + i];
            }
            parser->length = new_length;
            return;
        }
    }

    if (parser->frame[parser->length - 1U] == CS1237_HEADER_0)
    {
        parser->frame[0] = CS1237_HEADER_0;
        parser->length = 1U;
        return;
    }

    parser->length = 0U;
}

static void Cs1237_ParseByte(sim_uart_channel_t channel, uint8_t data)
{
    Cs1237FrameParser *parser;

    if (!SoftUart_ChannelValid(channel))
    {
        return;
    }

    parser = &s_cs1237_parsers[(uint32_t)channel];

    if (parser->length == 0U)
    {
        if (data == CS1237_HEADER_0)
        {
            parser->frame[0] = data;
            parser->length = 1U;
        }
        return;
    }

    if ((parser->length == 1U) && (data != CS1237_HEADER_1))
    {
        parser->length = (data == CS1237_HEADER_0) ? 1U : 0U;
        parser->frame[0] = CS1237_HEADER_0;
        return;
    }

    parser->frame[parser->length++] = data;
    if (parser->length < CS1237_FRAME_LENGTH)
    {
        return;
    }

    if (Cs1237_FrameValid(parser->frame))
    {
        Cs1237_UpdatePumpMessage(channel, parser->frame);
        parser->length = 0U;
    }
    else
    {
        /* 当前 21 字节候选帧无效时，从缓冲内部继续寻找下一处 AA55，避免粘包错位后长期丢帧。 */
        Cs1237_ParserResync(parser);
    }
}

/* 100ms 周期任务。
 * 职责为“搬运字节到队列”和“任务层解析完整协议帧”，不参与任何位级时序处理。 */
/**
 * @brief 模拟UART任务函数，处理UART数据传输
 * @param event 任务事件参数（此函数中未使用）
 */
static void SimUartTaskFunc(uint32_t event)
{
    uint8_t byte;
    uint8_t test_forward_buffer[SOFT_UART_ISR_BUFFER_SIZE];
    (void)event;

    if (!s_initialized)
    {
        return;
    }

    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        SoftUartChannelContext *ctx = &s_channels[i];
        uint16_t test_forward_length = 0U;
        sim_uart_channel_t channel = (sim_uart_channel_t)i;

        while (SoftUart_RingPopTask(channel, &byte))
        {
            Cs1237_ParseByte(channel, byte);

            if (xQueueSend(ctx->queue_handle, &byte, 0U) != pdPASS)
            {
                ctx->queue_overflow_count++;
            }

            /* 测试透传只复制被选中通道的原始字节，关闭时不会额外引入任何输出。 */
            if (SoftUart_TestForwardChannelSelected(channel) &&
                (test_forward_length < SOFT_UART_ISR_BUFFER_SIZE))
            {
                test_forward_buffer[test_forward_length++] = byte;
            }
        }

        if (test_forward_length > 0U)
        {
            SoftUart_TestForwardFrame(channel, test_forward_buffer, test_forward_length);
        }
    }

    Cs1237_ServicePumpLossTimeout(); /* 两路字节搬运结束后统一检查有效帧超时，处理拔泵无数据的离线边沿。 */
}

/* 初始化两路模拟串口的全部底层资源。
 * 该函数是幂等的，多次调用只会在第一次真正完成初始化。 */
/**
 * @brief 初始化所有模拟UART通道
 * @note 该函数会初始化所有模拟UART通道的GPIO、定时器、队列等资源
 * @note 如果已经初始化过，则直接返回
 */
void SimUart_InitAll(void)
{
    /* 检查是否已经初始化，避免重复初始化 */
    if (s_initialized)
    {
        return;
    }

    /* 使能GPIOE时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 初始化软定时器基准和接收定时器 */
    SoftUart_InitTimingBase();
    SoftUart_InitRxTimer();

    /* 遍历并初始化所有UART通道 */
    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        /* 获取当前通道的上下文结构体指针 */
        SoftUartChannelContext *ctx = &s_channels[i];

        /* 计算比特时间和半比特时间（单位：微秒） */
        ctx->bit_time_us = (uint16_t)(1000000UL / ctx->baudrate);
        ctx->half_bit_time_us = (uint16_t)(ctx->bit_time_us / 2U);

        /* 初始化环形缓冲区相关变量 */
        ctx->ring_head = 0U;
        ctx->ring_tail = 0U;
        ctx->ring_count = 0U;
        /* 初始化统计计数器 */
        ctx->received_bytes = 0U;
        ctx->queue_overflow_count = 0U;
        ctx->buffer_overflow_count = 0U;
        ctx->overlap_drop_count = 0U;
        ctx->framing_error_count = 0U;
        ctx->exti_suppressed = 0U;
        s_cs1237_parsers[i].length = 0U;

        /* 创建静态队列，用于存储接收到的数据 */
        ctx->queue_handle = xQueueCreateStatic(SOFT_UART_QUEUE_DEPTH,
                                               sizeof(uint8_t),
                                               ctx->queue_storage,
                                               &ctx->queue_control);
        configASSERT(ctx->queue_handle != NULL);  /* 确保队列创建成功 */

        /* 初始化通道GPIO */
        SoftUart_InitChannelGpio((sim_uart_channel_t)i);
    }

    /* 配置和使能外部中断线 */
    HAL_NVIC_SetPriority(EXTI4_IRQn, SOFT_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, SOFT_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* 重新配置外部中断线 */
    SoftUart_RearmExtiLines();
    /* 标记初始化完成 */
    s_initialized = 1U;
}

/* 创建后台搬运任务。
 * 对外暴露为单独初始化入口，便于在系统初始化阶段显式接入。 */
void SimUartTask_Init(void)
{
    SimUart_InitAll();

    if (s_task_created)
    {
        return;
    }

    Kernel_TaskCreate(&sSimUartTaskHandle, SimUartTaskFunc);
    Kernel_TaskStart(&sSimUartTaskHandle, KERNEL_TASK_ALWAYS, 100U);
    s_task_created = 1U;
}

/* 发送 1 字节。
 * 为保证位宽准确，发送期间临时关总中断，避免比特周期被其他中断拉长。 */
SoftUART_Status SimUart_SendByte(sim_uart_channel_t channel, uint8_t data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);
    uint32_t primask;

    if (ctx == NULL)
    {
        return SOFT_UART_ERROR;
    }

    SimUart_InitAll();

    if (s_receiver.active_channel != SIM_UART_NONE)
    {
        return SOFT_UART_BUSY;
    }

#if SOFT_UART_DIAG_LED
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#endif

    primask = __get_PRIMASK();
    __disable_irq();

    SoftUart_WriteTx(ctx, GPIO_PIN_RESET);
    delay_us(ctx->bit_time_us);

    for (uint8_t i = 0U; i < 8U; i++)
    {
        SoftUart_WriteTx(ctx, ((data & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data >>= 1;
        delay_us(ctx->bit_time_us);
    }

    SoftUart_WriteTx(ctx, GPIO_PIN_SET);
    delay_us(ctx->bit_time_us);

    if (primask == 0U)
    {
        __enable_irq();
    }

#if SOFT_UART_DIAG_LED
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
#endif

    return SOFT_UART_OK;
}

/* 逐字节发送缓冲区数据。 */
SoftUART_Status SimUart_Send(sim_uart_channel_t channel, const uint8_t *data, uint16_t len)
{
    if (data == NULL)
    {
        return SOFT_UART_ERROR;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        SoftUART_Status status = SimUart_SendByte(channel, data[i]);
        if (status != SOFT_UART_OK)
        {
            return status;
        }
    }

    return SOFT_UART_OK;
}

/* 发送 C 字符串，不包含字符串结束符。 */
SoftUART_Status SimUart_SendString(sim_uart_channel_t channel, const char *str)
{
    if (str == NULL)
    {
        return SOFT_UART_ERROR;
    }

    while (*str != '\0')
    {
        SoftUART_Status status = SimUart_SendByte(channel, (uint8_t)*str);
        if (status != SOFT_UART_OK)
        {
            return status;
        }
        str++;
    }

    return SOFT_UART_OK;
}

/* 从指定通道消息队列读取 1 字节。
 * 该函数是其他任务获取软串口数据的推荐入口。 */
SoftUART_Status SimUart_ReadByte(sim_uart_channel_t channel, uint8_t *data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if ((ctx == NULL) || (data == NULL))
    {
        return SOFT_UART_ERROR;
    }

    SimUart_InitAll();

    if (xQueueReceive(ctx->queue_handle, data, 0U) == pdPASS)
    {
        return SOFT_UART_OK;
    }

    return SOFT_UART_TIMEOUT;
}

/* 获取消息队列内当前累计的待处理字节数。 */
uint32_t SimUart_GetPendingBytes(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    if (!s_initialized)
    {
        return 0U;
    }

    if (ctx == NULL)
    {
        return 0U;
    }

    return (uint32_t)uxQueueMessagesWaiting(ctx->queue_handle);
}

/*
 * 函数功能：读取指定模拟串口通道的统计信息快照，供诊断或调试界面使用。
 * 输入参数：channel 为模拟串口通道；stats 为统计快照输出结构体指针。
 * 返回参数：无。
 */
void SimUart_GetStats(sim_uart_channel_t channel, SimUartStats *stats)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);
    uint32_t primask; /* 统计字段由高优先级软串口 ISR 更新，读取快照时需要用 PRIMASK 保证字段组合一致。 */

    if (!s_initialized)
    {
        return;
    }

    if ((ctx == NULL) || (stats == NULL))
    {
        return;
    }

    primask = __get_PRIMASK(); /* 软串口 IRQ 提升到 4 后不会被 FreeRTOS 临界区屏蔽，这里改用全局中断屏蔽。 */
    __disable_irq(); /* 暂停 TIM11/EXTI 写统计计数，避免读到一半时计数被 ISR 更新。 */
    stats->received_bytes = ctx->received_bytes;
    stats->queue_overflow_count = ctx->queue_overflow_count;
    stats->buffer_overflow_count = ctx->buffer_overflow_count;
    stats->overlap_drop_count = ctx->overlap_drop_count;
    stats->framing_error_count = ctx->framing_error_count;
    if (primask == 0U)
    {
        __enable_irq(); /* 进入快照前中断是打开状态才恢复，保持嵌套调用时的原始中断状态。 */
    }
}

/* 以下几个快捷函数用于读取单项统计值。 */
uint32_t SimUart_GetRxByteCount(sim_uart_channel_t channel)
{
    SimUartStats stats = {0};
    SimUart_GetStats(channel, &stats);
    return stats.received_bytes;
}

uint32_t SimUart_GetQueueOverflowCount(sim_uart_channel_t channel)
{
    SimUartStats stats = {0};
    SimUart_GetStats(channel, &stats);
    return stats.queue_overflow_count;
}

uint32_t SimUart_GetBufferOverflowCount(sim_uart_channel_t channel)
{
    SimUartStats stats = {0};
    SimUart_GetStats(channel, &stats);
    return stats.buffer_overflow_count;
}

uint32_t SimUart_GetOverlapDropCount(sim_uart_channel_t channel)
{
    SimUartStats stats = {0};
    SimUart_GetStats(channel, &stats);
    return stats.overlap_drop_count;
}

/* EXTI 起始位入口。
 * 若当前接收器空闲，则锁定该通道并启动半位延时采样；
 * 若另一通道正在接收，则仅增加 overlap 计数并抑制当前线重复中断。 */
void SimUart_HandleExti(uint16_t GPIO_Pin)
{
    sim_uart_channel_t channel;
    SoftUartChannelContext *ctx;

    if (!s_initialized)
    {
        return;
    }

    if (GPIO_Pin == s_channels[SIM_UART_1].rx_pin)
    {
        channel = SIM_UART_1;
    }
    else if (GPIO_Pin == s_channels[SIM_UART_2].rx_pin)
    {
        channel = SIM_UART_2;
    }
    else
    {
        return;
    }

    ctx = SoftUart_GetChannel(channel);
    if ((ctx == NULL) || (SoftUart_ReadRx(ctx) != GPIO_PIN_RESET))
    {
        return;
    }

    if (s_receiver.active_channel == SIM_UART_NONE)
    {
        s_receiver.active_channel = channel;
        s_receiver.stage = SOFT_UART_RX_STAGE_START;
        s_receiver.bit_index = 0U;
        s_receiver.current_byte = 0U;

        SoftUart_DisableExti(channel);
        SoftUart_StartSampleTimer(ctx->half_bit_time_us);
        return;
    }

    if (s_receiver.active_channel != channel)
    {
        ctx->overlap_drop_count++;
        if (!ctx->exti_suppressed)
        {
            SoftUart_DisableExti(channel);
            ctx->exti_suppressed = 1U;
        }
    }
}

/* TIM11 中断服务入口。
 * 只在确认更新中断有效时才推进状态机，避免误处理中断源。 */
void SimUart_TimerIrqHandler(void)
{
    if (!s_initialized)
    {
        return;
    }

    if ((SOFT_UART_TIMER_INSTANCE->DIER & TIM_DIER_UIE) == 0U)
    {
        return;
    }

    if ((SOFT_UART_TIMER_INSTANCE->SR & TIM_SR_UIF) == 0U)
    {
        return;
    }

    SOFT_UART_TIMER_INSTANCE->SR = 0U;
    SoftUart_ProcessSample();
}

/* 以下为历史兼容包装接口，统一映射到 SIM_UART_1。 */
SoftUART_Status Soft_UART_Init(void)
{
    SimUart_InitAll();
    return SOFT_UART_OK;
}

SoftUART_Status Soft_UART_SendByte(uint8_t data)
{
    return SimUart_SendByte(SIM_UART_1, data);
}

SoftUART_Status Soft_UART_ReceiveByte(uint8_t *data)
{
    return SimUart_ReadByte(SIM_UART_1, data);
}

SoftUART_Status Soft_UART_Send(uint8_t *data, uint16_t len)
{
    return SimUart_Send(SIM_UART_1, data, len);
}

SoftUART_Status Soft_UART_Receive(uint8_t *data, uint16_t len)
{
    if (data == NULL)
    {
        return SOFT_UART_ERROR;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        SoftUART_Status status = SimUart_ReadByte(SIM_UART_1, &data[i]);
        if (status != SOFT_UART_OK)
        {
            if ((i == 0U) && (status == SOFT_UART_TIMEOUT))
            {
                return SOFT_UART_TIMEOUT;
            }
            return status;
        }
    }

    return SOFT_UART_OK;
}

SoftUART_Status Soft_UART_SendString(char *str)
{
    return SimUart_SendString(SIM_UART_1, str);
}
