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
#define SOFT_UART_IRQ_PRIORITY         4U  /* 两路位采样都高于 FreeRTOS 屏蔽阈值 5，避免 TIM11/TIM13 被任务临界区延迟导致错帧。 */
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
#define CS1237_DEVICE_CODE_INJECT_WATER 0x07U  /* 压力板上报 0x07 时，主控识别为注水泵。 */
#define CS1237_DEVICE_CODE_POUR_WATER   0x0EU  /* 压力板上报 0x0E 时，主控识别为灌注泵。 */
#define CS1237_DEVICE_CODE_DRAW_WATER   0x0DU  /* 压力板上报 0x0D 时，主控识别为抽吸泵。 */
#define CS1237_PUMP_LOSS_SUSPECT_MS     1500U  /* 超过 1.5 秒无有效帧先进入疑似丢失，避免单次软串口错帧立刻清在线状态。 */
#define CS1237_PUMP_LOSS_CONFIRM_MS     3000U  /* 疑似丢失持续到 3 秒仍无有效帧才确认离线，兼顾拔泵响应和偶发错帧容错。 */
#define CS1237_RAW_MIN_VALUE            (-8388608L) /* CS1237原始值必须是24位二进制补码符号扩展后的最小值。 */
#define CS1237_RAW_MAX_VALUE            8388607L /* CS1237原始值必须是24位二进制补码符号扩展后的最大值。 */
#define CS1237_WEIGHT_MAX_X10           500000U /* 重量最大接受50000.0g，与压力板现有50000g配置上限一致。 */
#define CS1237_THRESHOLD_MAX_G          50000U /* 阈值上限复用压力板存储边界，避免CRC正确的异常大值进入业务状态。 */
#define CS1237_RECOVERY_VALID_FRAMES    3U /* 发现业务异常后必须连续收到3帧合理数据，才重新信任压力链路。 */

/* 位级接收状态机阶段定义。
 * 采用“起始位确认 -> 8 位数据 -> 停止位确认”的 8N1 接收流程。 */
typedef enum {
    SOFT_UART_RX_STAGE_IDLE = 0,
    SOFT_UART_RX_STAGE_START,
    SOFT_UART_RX_STAGE_DATA,
    SOFT_UART_RX_STAGE_STOP
} SoftUartRxStage;

/* 单通道上下文。
 * 每路都保存自身 GPIO、采样定时器、接收状态、消息缓冲和诊断计数；
 * A/B 两路同时上报时，各自推进自己的接收状态，不再互相丢弃起始位。 */
typedef struct {
    GPIO_TypeDef *tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef *rx_port;
    uint16_t rx_pin;
    uint32_t baudrate;
    uint16_t bit_time_us;
    uint16_t half_bit_time_us;

    TIM_TypeDef *rx_timer;             /* 本通道独占的位采样定时器；PE4 用 TIM11，PE6 用 TIM13。 */
    IRQn_Type rx_timer_irq;            /* 本通道采样定时器对应的 NVIC 中断号。 */
    uint8_t rx_timer_on_apb2;          /* 1 表示定时器在 APB2，0 表示在 APB1，用于计算真实 1MHz 预分频。 */
    volatile SoftUartRxStage rx_stage; /* 本通道独立接收阶段，两路同时发帧时互不抢占。 */
    volatile uint8_t rx_bit_index;     /* 本通道当前正在接收的数据位序号。 */
    volatile uint8_t rx_current_byte;  /* 本通道正在拼装的 8 位数据。 */

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

} SoftUartChannelContext;

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
        .rx_timer = TIM11,
        .rx_timer_irq = TIM1_TRG_COM_TIM11_IRQn,
        .rx_timer_on_apb2 = 1U,
        .rx_stage = SOFT_UART_RX_STAGE_IDLE,
    },
    {
        .tx_port = GPIOE,
        .tx_pin = GPIO_PIN_11,
        .rx_port = GPIOE,
        .rx_pin = GPIO_PIN_6,
        .baudrate = SOFT_UART_DEFAULT_BAUDRATE,
        .rx_timer = TIM13,
        .rx_timer_irq = TIM8_UP_TIM13_IRQn,
        .rx_timer_on_apb2 = 0U,
        .rx_stage = SOFT_UART_RX_STAGE_IDLE,
    }
};

static kernel_task_t sSimUartTaskHandle;
static uint8_t s_initialized = 0U;
static uint8_t s_task_created = 0U;
static uint8_t s_dwt_ready = 0U;
static uint8_t s_tim7_ready = 0U;
static uint8_t s_rx_timers_initialized = 0U;
static Cs1237FrameParser s_cs1237_parsers[SIM_UART_COUNT];
static uint32_t s_cs1237_last_valid_tick[SIM_UART_COUNT]; /* 记录每路最近一次有效帧时间，用于拔泵后无数据场景的离线判定。 */
static uint32_t s_cs1237_loss_suspect_tick[SIM_UART_COUNT]; /* 记录每路首次进入疑似丢失的时间，用于连续确认后再清在线状态。 */
static uint8_t s_cs1237_recovery_valid_count[SIM_UART_COUNT]; /* 业务异常后记录每路连续合理帧数，防止单帧恢复再次制造尖峰。 */
static bool s_cs1237_recovery_required[SIM_UART_COUNT]; /* true表示该路刚出现业务越界，必须完成连续合理帧确认。 */

/* 私有函数声明区：
 * 这些函数按“时基/硬件控制 -> 缓冲区 -> 状态机 -> 对外接口”的顺序组织。 */
static void SoftUart_InitTimingBase(void);
static void SoftUart_InitRxTimers(void);
static void SoftUart_InitChannelGpio(sim_uart_channel_t channel);
static void SoftUart_StartSampleTimer(SoftUartChannelContext *ctx, uint16_t delay_us);
static void SoftUart_StopSampleTimer(SoftUartChannelContext *ctx);
static void SoftUart_EnableExti(sim_uart_channel_t channel);
static void SoftUart_DisableExti(sim_uart_channel_t channel);
static void SoftUart_RingPushFromIsr(sim_uart_channel_t channel, uint8_t data);
static bool SoftUart_RingPopTask(sim_uart_channel_t channel, uint8_t *data);
static void SoftUart_FinishReceive(sim_uart_channel_t channel, bool byte_valid);
static void SoftUart_ProcessSample(sim_uart_channel_t channel);
static void SimUartTaskFunc(uint32_t event);
static SoftUartChannelContext *SoftUart_GetChannel(sim_uart_channel_t channel);
static uint32_t SoftUart_GetTimerClockHz(const SoftUartChannelContext *ctx);
static bool SoftUart_AnyReceiverBusy(void);
static void SoftUart_WriteTx(const SoftUartChannelContext *channel, GPIO_PinState level);
static GPIO_PinState SoftUart_ReadRx(const SoftUartChannelContext *channel);
static bool SoftUart_IsForwardSource(sim_uart_channel_t channel);
static void SoftUart_TestForwardFrame(sim_uart_channel_t channel, const uint8_t *data, uint16_t length);
static uint16_t Cs1237_CalcCrc16Modbus(const uint8_t *data, uint16_t length);
static uint16_t Cs1237_ReadU16Le(const uint8_t *data);
static uint32_t Cs1237_ReadU32Le(const uint8_t *data);
static bool Cs1237_FrameValid(const uint8_t *frame);
static bool Cs1237_BusinessDataTrusted(sim_uart_channel_t channel, const uint8_t *frame);
static pumpMessage_t *Cs1237_GetPump(sim_uart_channel_t channel);
static void Cs1237_BeepOnceIfNoAlarm(void);
static void Cs1237_RefreshPumpUi(sim_uart_channel_t channel);
static void Cs1237_UpdatePumpMessage(sim_uart_channel_t channel, const uint8_t *frame);
static void Cs1237_CheckPumpTimeout(void);
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
    /* 通道号越界时拒绝取数组元素，避免后续访问错误 GPIO、定时器和队列。 */
    if (!SoftUart_ChannelValid(channel))
    {
        return NULL;
    }

    return &s_channels[(uint32_t)channel];
}

/*
 * 函数功能：检查两路软串口是否有任一路正在接收，用于发送前避免 TX 关中断破坏 RX 采样。
 * 输入参数：无。
 * 返回参数：任一路不在空闲阶段时返回 true，两路都空闲时返回 false。
 */
static bool SoftUart_AnyReceiverBusy(void)
{
    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        if (s_channels[i].rx_stage != SOFT_UART_RX_STAGE_IDLE)
        {
            return true; /* 任一路正在收起始位、数据位或停止位时都不能执行阻塞式发送。 */
        }
    }

    return false; /* 两路都空闲，发送侧可安全临时关中断输出。 */
}

/*
 * 函数功能：按模拟串口通道取得对应的 A/B 泵公共状态结构。
 * 输入参数：channel 为压力模块模拟串口通道，当前线束要求 SIM_UART_1/PE4 接 A 泵压力传感器，SIM_UART_2/PE6 接 B 泵压力传感器。
 * 返回参数：有效通道返回对应 pumpMessage_t 指针，非法通道返回 NULL。
 */
static pumpMessage_t *Cs1237_GetPump(sim_uart_channel_t channel)
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
static void Cs1237_RefreshPumpUi(sim_uart_channel_t channel)
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
static bool SoftUart_IsForwardSource(sim_uart_channel_t channel)
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
    /* 缓冲为空或长度为 0 时没有可透传内容，保持测试串口静默。 */
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    /* 仅透传配置选中的压力通道，避免 A/B 原始字节混在同一测试输出中。 */
    if (!SoftUart_IsForwardSource(channel))
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

    /* 任一微秒计时源已经就绪时不重复初始化，避免重置正在使用的计数器。 */
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

    /* 芯片未开放 DWT 时再启用 TIM7 作为微秒延时后备，保证位时序仍可用。 */
    if (!s_dwt_ready)
    {
        start = __HAL_TIM_GET_COUNTER(&htim7);
        for (volatile uint32_t i = 0U; i < 1000U; i++)
        {
        }
        s_tim7_ready = (__HAL_TIM_GET_COUNTER(&htim7) != start) ? 1U : 0U;
    }
}

/*
 * 函数功能：为软串口 TX 位发送提供微秒延时；RX 由 A 路 TIM11、B 路 TIM13 独立中断采样，不使用本函数阻塞接收。
 * 输入参数：us 为需要等待的微秒数。
 * 返回参数：无。
 */
static void delay_us(uint32_t us)
{
    /* DWT 可用时优先按 CPU 周期延时，提供软件 UART 最稳定的位宽。 */
    if (s_dwt_ready)
    {
        uint32_t start = DWT->CYCCNT;
        uint32_t cycles = us * (SystemCoreClock / 1000000UL);

        while ((DWT->CYCCNT - start) < cycles)
        {
        }
        return;
    }

    /* DWT 不可用但 TIM7 已就绪时，用 1MHz 定时器完成同等微秒等待。 */
    if (s_tim7_ready)
    {
        uint32_t remaining = us;
        uint32_t start = __HAL_TIM_GET_COUNTER(&htim7);

        while (remaining > 0U)
        {
            uint32_t now = __HAL_TIM_GET_COUNTER(&htim7);
            uint32_t elapsed = (now >= start) ? (now - start) : (0x10000U + now - start);
            /* 已累计到本段所需微秒数时结束轮询，继续发送或采样下一位。 */
            if (elapsed >= remaining)
            {
                break;
            }
            remaining -= elapsed;
            start = now;
        }
        return;
    }

    /* 两个微秒计时源都不可用时，整毫秒部分退化为 HAL 延时。 */
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

/*
 * 函数功能：取得指定通道采样定时器的真实输入时钟，用于配置 1MHz 微秒计数。
 * 输入参数：ctx 为通道上下文，内部用 rx_timer_on_apb2 区分 TIM11 和 TIM13 所在总线。
 * 返回参数：定时器输入时钟，单位 Hz；参数为空时返回 0。
 */
static uint32_t SoftUart_GetTimerClockHz(const SoftUartChannelContext *ctx)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t pFLatency = 0U;
    uint32_t apb_prescaler;
    uint32_t tim_clock;

    if (ctx == NULL)
    {
        return 0U; /* 非法通道不能配置定时器，避免后续除零或写错外设。 */
    }

    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
    if (ctx->rx_timer_on_apb2 != 0U)
    {
        apb_prescaler = clkconfig.APB2CLKDivider; /* TIM11 位于 APB2，读取 APB2 分频和外设时钟。 */
        tim_clock = HAL_RCC_GetPCLK2Freq();       /* APB2 未分频时定时器时钟等于 PCLK2。 */
    }
    else
    {
        apb_prescaler = clkconfig.APB1CLKDivider; /* TIM13 位于 APB1，不能继续沿用 TIM11 的 APB2 时钟。 */
        tim_clock = HAL_RCC_GetPCLK1Freq();       /* APB1 未分频时定时器时钟等于 PCLK1。 */
    }

    if (apb_prescaler != RCC_HCLK_DIV1)
    {
        tim_clock *= 2UL; /* STM32 定时器所在 APB 分频不为 1 时，定时器时钟自动乘 2。 */
    }

    return tim_clock;
}

/*
 * 函数功能：初始化两路独立 RX 采样定时器，PE4 使用 TIM11，PE6 使用 TIM13。
 * 输入参数：无。
 * 返回参数：无。
 */
static void SoftUart_InitRxTimers(void)
{
    if (s_rx_timers_initialized != 0U)
    {
        return; /* 两路定时器只初始化一次，重复调用不打断正在运行的接收状态。 */
    }

    /* 两个定时器分别属于 APB2 和 APB1，均只作为内部单次计时器，不占用 GPIO 复用功能。 */
    __HAL_RCC_TIM11_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();

    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        SoftUartChannelContext *ctx = &s_channels[i]; /* 每次只配置当前通道独占的采样定时器。 */
        uint32_t timer_clock_hz = SoftUart_GetTimerClockHz(ctx); /* 按所在 APB 取得真实定时器时钟。 */
        uint32_t prescaler = timer_clock_hz / 1000000U;          /* 预分频到 1MHz，使 ARR 直接使用微秒。 */

        if (prescaler == 0U)
        {
            prescaler = 1U; /* 防御异常时钟配置，避免 PSC 无符号下溢。 */
        }

        ctx->rx_timer->CR1 = 0U;                              /* 初始化时保持定时器停止。 */
        ctx->rx_timer->PSC = (uint16_t)(prescaler - 1U);      /* 装载当前总线对应的 1MHz 预分频。 */
        ctx->rx_timer->ARR = 0xFFFFU;                         /* 空闲期保持最大重装值，实际接收时按半位/一位覆盖。 */
        ctx->rx_timer->CNT = 0U;                              /* 清计数器，避免继承上电随机值。 */
        ctx->rx_timer->SR = 0U;                               /* 清历史更新标志。 */
        ctx->rx_timer->DIER = 0U;                             /* 起始位到来前不允许更新中断。 */
        ctx->rx_timer->EGR = TIM_EGR_UG;                      /* 立即把 PSC/ARR 写入影子寄存器。 */
        ctx->rx_timer->SR = 0U;                               /* UG 会产生更新标志，必须再次清除。 */

        HAL_NVIC_SetPriority(ctx->rx_timer_irq, SOFT_UART_IRQ_PRIORITY, 0U); /* 两路采样保持同一高优先级。 */
        HAL_NVIC_EnableIRQ(ctx->rx_timer_irq);                 /* 允许本通道独立采样中断。 */
    }

    s_rx_timers_initialized = 1U; /* 两路都配置完成后再置位，避免只初始化一半。 */
}

/*
 * 函数功能：启动当前通道的一次单次采样计时，起始位用半位时间，后续使用一位时间。
 * 输入参数：ctx 为当前通道上下文；delay_us 为本次等待微秒数。
 * 返回参数：无。
 */
static void SoftUart_StartSampleTimer(SoftUartChannelContext *ctx, uint16_t delay_us)
{
    if (ctx == NULL)
    {
        return; /* 非法通道没有定时器可写，保持其它通道接收不受影响。 */
    }

    if (delay_us == 0U)
    {
        delay_us = 1U; /* ARR 不能装载无符号负值，最短等待固定为 1 微秒。 */
    }

    ctx->rx_timer->CR1 = 0U;                                  /* 重新装载前先停当前通道定时器。 */
    ctx->rx_timer->CNT = 0U;                                  /* 每次采样等待都从零开始。 */
    ctx->rx_timer->ARR = (uint32_t)delay_us - 1U;              /* 1MHz 下 ARR 直接对应等待微秒数减一。 */
    ctx->rx_timer->EGR = TIM_EGR_UG;                           /* 立即应用新的重装值。 */
    ctx->rx_timer->SR = 0U;                                    /* 清 UG 产生的更新标志。 */
    ctx->rx_timer->DIER = TIM_DIER_UIE;                        /* 只开放更新中断。 */
    ctx->rx_timer->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;            /* 单脉冲模式到点自动停止。 */
}

/*
 * 函数功能：停止当前通道采样定时器并清中断状态，结束本字节接收。
 * 输入参数：ctx 为当前通道上下文。
 * 返回参数：无。
 */
static void SoftUart_StopSampleTimer(SoftUartChannelContext *ctx)
{
    if (ctx == NULL)
    {
        return; /* 空通道无需停止，避免访问非法定时器地址。 */
    }

    ctx->rx_timer->CR1 &= ~TIM_CR1_CEN; /* 只停止本通道定时器，另一通道可继续采样。 */
    ctx->rx_timer->DIER = 0U;            /* 关闭本通道更新中断。 */
    ctx->rx_timer->SR = 0U;              /* 清除本通道剩余更新标志。 */
}

/*
 * 函数功能：开放指定通道的下降沿中断，使该通道可以捕获下一个串口起始位。
 * 输入参数：channel 为需要恢复起始位检测的模拟串口通道。
 * 返回参数：无。
 */
static void SoftUart_EnableExti(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel); /* 只获取指定通道，不能改动另一通道的中断线。 */

    if (ctx == NULL)
    {
        return; /* 非法通道没有可开放的接收引脚，直接退出。 */
    }

    __HAL_GPIO_EXTI_CLEAR_IT(ctx->rx_pin); /* 先清历史下降沿，避免刚开放就误进入接收状态。 */
    EXTI->IMR |= ctx->rx_pin;             /* 只开放本通道起始位检测，另一通道保持原运行状态。 */
}

/*
 * 函数功能：关闭指定通道的下降沿中断，避免接收数据位时被重复当成起始位。
 * 输入参数：channel 为当前正在接收字节的模拟串口通道。
 * 返回参数：无。
 */
static void SoftUart_DisableExti(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel); /* 只关闭当前通道，不能阻断另一泵的并行上报。 */

    if (ctx == NULL)
    {
        return; /* 非法通道没有可关闭的接收引脚，直接退出。 */
    }

    EXTI->IMR &= ~(uint32_t)ctx->rx_pin;   /* 接收本字节期间屏蔽本通道后续下降沿。 */
    __HAL_GPIO_EXTI_CLEAR_IT(ctx->rx_pin); /* 清本通道残留标志，避免字节结束后产生假起始位。 */
}

/* 初始化单路 GPIO。
 * TX 为推挽输出并默认拉高空闲，RX 采用下降沿 EXTI 检测起始位。 */
static void SoftUart_InitChannelGpio(sim_uart_channel_t channel)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    /* 非法通道没有固定 GPIO 映射，不能配置未知引脚或改变其它外设。 */
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

    /* 非法通道没有环形缓冲，不能在中断中写入未知内存。 */
    if (ctx == NULL)
    {
        return;
    }

    /* 环形缓冲已满时只记录溢出并丢弃新字节，禁止覆盖尚未搬走的压力数据。 */
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

    /* 通道或输出指针无效时不能安全取字节，直接报告缓冲无数据。 */
    if ((ctx == NULL) || (data == NULL))
    {
        return false;
    }

    primask = __get_PRIMASK(); /* 软串口 ISR 优先级高于 FreeRTOS 屏蔽阈值，BASEPRI 不能保护下面的环形缓冲共享索引。 */
    __disable_irq(); /* 暂停所有中断，防止 TIM11/TIM13 ISR 写索引时任务同时修改同一通道计数。 */
    /* 缓冲中确有数据时才推进尾索引，空缓冲保持索引和计数不变。 */
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

/*
 * 函数功能：结束指定通道的一次字节接收；停止位正确时提交字节，错误时只清状态。
 * 输入参数：channel 为本次接收所属通道；byte_valid 表示停止位是否有效。
 * 返回参数：无。
 */
static void SoftUart_FinishReceive(sim_uart_channel_t channel, bool byte_valid)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel); /* 只结束触发本次中断的通道，不能重置另一通道。 */

    if (ctx == NULL)
    {
        return; /* 非法通道没有接收状态可结束。 */
    }

    if (byte_valid)
    {
        SoftUart_RingPushFromIsr(channel, ctx->rx_current_byte); /* 停止位有效才把本通道完整字节写入环形缓冲。 */
    }

    SoftUart_StopSampleTimer(ctx);              /* 停止本通道采样，不影响另一通道正在运行的定时器。 */
    ctx->rx_stage = SOFT_UART_RX_STAGE_IDLE;    /* 本通道回到等待下一个起始位状态。 */
    ctx->rx_bit_index = 0U;                     /* 清本通道数据位序号。 */
    ctx->rx_current_byte = 0U;                  /* 清本通道临时字节。 */
    SoftUart_EnableExti(channel);                /* 只重新开放本通道起始位，另一通道保持自己的接收状态。 */
}

/*
 * 函数功能：推进指定通道的位级接收状态机，TIM11/TIM13 每次中断各处理自己的一个采样点。
 * 输入参数：channel 为本次采样所属的模拟串口通道。
 * 返回参数：无。
 */
static void SoftUart_ProcessSample(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel); /* 由独立定时器中断明确选择通道。 */
    GPIO_PinState pin_state;

    /* 非法通道没有接收上下文，定时中断不能推进任何状态机。 */
    if (ctx == NULL)
    {
        return;
    }

    pin_state = SoftUart_ReadRx(ctx);

    switch (ctx->rx_stage)
    {
        case SOFT_UART_RX_STAGE_START:
            if (pin_state != GPIO_PIN_RESET)
            {
                ctx->framing_error_count++;
                SoftUart_FinishReceive(channel, false); /* 半位处已经回高说明不是有效起始位，只结束本通道。 */
                return;
            }

            ctx->rx_stage = SOFT_UART_RX_STAGE_DATA; /* 起始位有效，进入本通道 8 位数据采样。 */
            ctx->rx_bit_index = 0U;                  /* 第一位从 bit0 开始。 */
            ctx->rx_current_byte = 0U;               /* 新字节开始前清临时值。 */
            SoftUart_StartSampleTimer(ctx, ctx->bit_time_us); /* 下一次在 bit0 中心采样。 */
            break;

        case SOFT_UART_RX_STAGE_DATA:
            if (pin_state != GPIO_PIN_RESET)
            {
                ctx->rx_current_byte |= (uint8_t)(1U << ctx->rx_bit_index); /* UART 低位先发，当前高电平写入对应位。 */
            }

            ctx->rx_bit_index++; /* 推进到下一数据位。 */
            if (ctx->rx_bit_index >= 8U)
            {
                ctx->rx_stage = SOFT_UART_RX_STAGE_STOP; /* 8 位收完后下一采样点检查停止位。 */
            }

            SoftUart_StartSampleTimer(ctx, ctx->bit_time_us); /* 按一位时间继续本通道采样。 */
            break;

        case SOFT_UART_RX_STAGE_STOP:
            if (pin_state == GPIO_PIN_SET)
            {
                SoftUart_FinishReceive(channel, true); /* 停止位为高，提交本通道完整字节。 */
            }
            else
            {
                ctx->framing_error_count++;
                SoftUart_FinishReceive(channel, false); /* 停止位为低，丢弃本通道错误字节。 */
            }
            break;

        case SOFT_UART_RX_STAGE_IDLE:
        default:
            SoftUart_FinishReceive(channel, false); /* 空闲状态误进定时器中断时只清本通道残留。 */
            break;
    }
}

/*
 * 函数功能：按 CRC16/MODBUS 规则计算压力上报帧校验值，防止错误字节进入泵状态。
 * 输入参数：data 为待校验数据起始地址；length 为参与校验的字节数。
 * 返回参数：计算得到的 16 位 CRC，低字节在压力帧中先发送。
 */
static uint16_t Cs1237_CalcCrc16Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            /* CRC 最低位为 1 时右移后异或 MODBUS 多项式，否则只右移。 */
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

/*
 * 函数功能：从压力帧中读取一个低字节在前的 16 位无符号数。
 * 输入参数：data 指向字段的低字节。
 * 返回参数：按小端顺序组合后的 16 位数值。
 */
static uint16_t Cs1237_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U);
}

/*
 * 函数功能：从压力帧中读取一个低字节在前的 32 位无符号数。
 * 输入参数：data 指向字段的最低字节。
 * 返回参数：按小端顺序组合后的 32 位数值。
 */
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

    /* 协议版本、消息类型或载荷长度任一不符时，候选数据不能作为压力上报帧。 */
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
 * 函数功能：检查CRC正确的压力帧是否符合CS1237硬件和压力板业务范围，并管理异常后的连续恢复确认。
 * 输入参数：channel 为A/B压力软串口通道；frame 指向已通过固定字段和CRC校验的21字节帧。
 * 返回参数：当前帧可以刷新泵业务状态时返回true；越界或仍在连续恢复确认阶段时返回false。
 */
static bool Cs1237_BusinessDataTrusted(sim_uart_channel_t channel, const uint8_t *frame)
{
    uint32_t channel_index; /* 通道枚举直接对应A/B两路恢复状态数组下标。 */
    int32_t raw_cs1237; /* 保存24位符号扩展后的原始ADC值，用于拒绝字段错位产生的32位尖峰。 */
    uint32_t weight_x10; /* 保存0.1g单位重量，用于限制压力板支持的最大业务范围。 */
    uint16_t threshold_g; /* 保存g单位阈值，用于拒绝超过压力板配置上限的异常值。 */
    bool data_in_range; /* 汇总本帧三个数值字段和设备码高位是否都满足协议业务约束。 */

    if (SoftUart_ChannelValid(channel) == false)
    {
        return false; /* 非法通道没有独立恢复状态，不能把数据写入任一泵。 */
    }

    channel_index = (uint32_t)channel; /* 通道合法后再转换下标，避免数组越界。 */
    raw_cs1237 = (int32_t)Cs1237_ReadU32Le(&frame[6]); /* 按协议小端读取RawCs1237并解释为有符号值。 */
    weight_x10 = Cs1237_ReadU32Le(&frame[10]); /* 按协议小端读取最终重量，单位保持0.1g。 */
    threshold_g = Cs1237_ReadU16Le(&frame[14]); /* 按协议小端读取压力阈值，单位保持g。 */
    data_in_range = (raw_cs1237 >= CS1237_RAW_MIN_VALUE) &&
                    (raw_cs1237 <= CS1237_RAW_MAX_VALUE) &&
                    (weight_x10 <= CS1237_WEIGHT_MAX_X10) &&
                    (threshold_g <= CS1237_THRESHOLD_MAX_G) &&
                    ((frame[16] & 0xF0U) == 0U); /* 设备码只允许使用PA1~PA4对应的低4位，高位污染说明载荷不可信。 */

    if (data_in_range == false)
    {
        s_cs1237_recovery_required[channel_index] = true; /* 记录该路出现业务异常，后续单个正常帧不能立即恢复。 */
        s_cs1237_recovery_valid_count[channel_index] = 0U; /* 新异常会打断已有恢复计数，必须重新连续确认。 */
        return false; /* 越界帧不刷新在线时间、压力闭环、UI或外控上传数据。 */
    }

    if (s_cs1237_recovery_required[channel_index] == false)
    {
        return true; /* 链路此前可信时，当前合理帧沿用原有单帧实时更新行为。 */
    }

    if (s_cs1237_recovery_valid_count[channel_index] < CS1237_RECOVERY_VALID_FRAMES)
    {
        s_cs1237_recovery_valid_count[channel_index]++; /* 每个连续合理帧只累计一次恢复确认。 */
    }
    if (s_cs1237_recovery_valid_count[channel_index] < CS1237_RECOVERY_VALID_FRAMES)
    {
        return false; /* 尚未连续达到3帧时保留最后可信压力状态，避免异常和恢复值来回闪动。 */
    }

    s_cs1237_recovery_required[channel_index] = false; /* 第3个连续合理帧到达后恢复该路业务更新。 */
    s_cs1237_recovery_valid_count[channel_index] = 0U; /* 恢复完成后清计数，下一次异常重新开始。 */
    return true; /* 当前第3帧同时作为恢复后的第一帧可信业务数据写入。 */
}

/*
 * 函数功能：把 CS1237 模拟串口帧中的设备码转换成业务泵类型。
 * 输入参数：device_code 为下位机上报帧第 16 字节设备码。
 * 返回参数：DRAWWATER/INJECTWATER/POURWATER 表示已识别泵类型，0 表示备用码或未知码。
 */
static uint16_t Cs1237_DecodePumpType(uint8_t device_code)
{
    /* 设备码只在这里转换为业务类型，避免外部通信或手柄联动路径再固定覆盖泵类型。 */
    switch (device_code)
    {
        case CS1237_DEVICE_CODE_INJECT_WATER:
            return INJECTWATER; /* 0x07 明确识别为注水泵，允许手柄冷却联动。 */
        case CS1237_DEVICE_CODE_POUR_WATER:
            return POURWATER; /* 0x0E 明确识别为灌注泵，不参与手柄冷却联动。 */
        case CS1237_DEVICE_CODE_DRAW_WATER:
            return DRAWWATER; /* 0x0D 明确识别为抽吸泵，不参与手柄冷却联动。 */
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
    uint16_t pump_type = Cs1237_DecodePumpType(device_code);
    bool new_online_flag = (pump_type != 0U);
    uint16_t old_pump_type;
    bool old_online_flag;
    bool pump_online_changed;
    bool pump_display_changed;
    uint32_t now_tick = HAL_GetTick();

    pump_message = Cs1237_GetPump(channel);
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

    /* 泵类型或在线状态发生变化时才重装默认速度并刷新对应泵区，避免每帧覆盖用户设定。 */
    if (pump_display_changed)
    {
        if(pump_message->type==INJECTWATER)
        {
        pump_message->speed_work = 30U; /* 注水泵设备码识别成功后固定装载 30 作为本次上线默认流量，后续调速和手柄配置流程保持不变。 */
        }
        else if(pump_message->type==POURWATER)
        {
        pump_message->speed_work=200; /* 灌注泵刚识别时装载 200 档默认设定，后续屏幕调速从该值继续。 */
        }
        else if(pump_message->type==DRAWWATER)
        {
        pump_message->speed_work=10; /* 抽水泵刚识别时装载 10 档默认设定，避免上线后保持零速。 */
        }
        Cs1237_RefreshPumpUi(channel); /* 泵类型或在线状态变化后立即刷新对应泵区，保证屏幕可用态同步。 */
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
static void Cs1237_CheckPumpTimeout(void)
{
    uint32_t now_tick = HAL_GetTick();

    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        sim_uart_channel_t channel = (sim_uart_channel_t)i;
        pumpMessage_t *pump_message = Cs1237_GetPump(channel);
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
            Cs1237_RefreshPumpUi(channel); /* 离线状态已经写入公共结构，立即把对应泵区刷为不可用。 */
            Cs1237_BeepOnceIfNoAlarm(); /* 泵丢失边沿蜂鸣一次，报警期间不抢占报警蜂鸣。 */
        }
    }
}

/*
 * 函数功能：压力候选帧校验失败后，在现有缓存内寻找下一处 AA55 帧头并恢复解析位置。
 * 输入参数：parser 为当前通道的压力帧解析上下文。
 * 返回参数：无；函数直接更新缓存内容和有效长度。
 */
static void Cs1237_ParserResync(Cs1237FrameParser *parser)
{
    uint8_t new_length = 0U;

    for (uint8_t start = 1U; (start + 1U) < parser->length; start++)
    {
        if ((parser->frame[start] == CS1237_HEADER_0) &&
            (parser->frame[start + 1U] == CS1237_HEADER_1))
        {
            new_length = (uint8_t)(parser->length - start); /* 缓存内部已有下一帧头时，只保留该帧头之后的数据，避免粘包错位后整段丢弃。 */
            for (uint8_t i = 0U; i < new_length; i++)
            {
                parser->frame[i] = parser->frame[start + i]; /* 把下一候选帧移到缓冲起点，后续字节可直接续接。 */
            }
            parser->length = new_length; /* 新长度只覆盖已经保留下来的候选帧内容。 */
            return;
        }
    }

    if (parser->frame[parser->length - 1U] == CS1237_HEADER_0)
    {
        parser->frame[0] = CS1237_HEADER_0; /* 末字节可能是下一帧的第一个 AA，保留它等待后续 55。 */
        parser->length = 1U; /* 解析器回到“已收到第一个帧头字节”状态。 */
        return;
    }

    parser->length = 0U; /* 缓存中没有可复用帧头时彻底清空，从下一字节重新找 AA。 */
}

/*
 * 函数功能：逐字节组装指定通道的 21 字节压力帧，并在满帧后校验、发布或重同步。
 * 输入参数：channel 为字节所属模拟串口通道；data 为本次收到的原始字节。
 * 返回参数：无。
 */
static void Cs1237_ParseByte(sim_uart_channel_t channel, uint8_t data)
{
    Cs1237FrameParser *parser;

    if (!SoftUart_ChannelValid(channel))
    {
        return; /* 非法通道没有独立解析缓存，直接拒绝以防数组越界。 */
    }

    parser = &s_cs1237_parsers[(uint32_t)channel];

    /* 当前没有候选帧时只查找第一个 AA 帧头，普通噪声不能进入解析缓存。 */
    if (parser->length == 0U)
    {
        if (data == CS1237_HEADER_0)
        {
            parser->frame[0] = data; /* 空闲状态只接受 AA 作为候选帧起点，普通噪声字节直接忽略。 */
            parser->length = 1U; /* 记录已收到第一帧头字节，下一字节必须为 55。 */
        }
        return;
    }

    if ((parser->length == 1U) && (data != CS1237_HEADER_1))
    {
        parser->length = (data == CS1237_HEADER_0) ? 1U : 0U; /* 第二字节不是 55 时，重复 AA 可继续作为新帧头，其它值则回到空闲。 */
        parser->frame[0] = CS1237_HEADER_0; /* 统一保留候选 AA，避免连续 AA55 的有效帧被漏掉。 */
        return;
    }

    parser->frame[parser->length++] = data; /* 帧头已确认后按接收顺序保存协议字节。 */
    if (parser->length < CS1237_FRAME_LENGTH)
    {
        return; /* 21 字节尚未收齐时继续等待，不能提前刷新泵在线和压力状态。 */
    }

    if (Cs1237_FrameValid(parser->frame))
    {
        if (Cs1237_BusinessDataTrusted(channel, parser->frame))
        {
            Cs1237_UpdatePumpMessage(channel, parser->frame); /* 结构、CRC和业务范围全部可信后才写入泵状态。 */
        }
        parser->length = 0U; /* 本帧已经消费，清长度等待下一帧。 */
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
        return; /* 队列和通道上下文尚未初始化时不能搬运字节，避免访问空队列句柄。 */
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
                ctx->queue_overflow_count++; /* 压力协议已在入队前解析；兼容字节队列满时只记录诊断计数，不回退泵状态。 */
            }

            /* 测试透传只复制被选中通道的原始字节，关闭时不会额外引入任何输出。 */
            if (SoftUart_IsForwardSource(channel) &&
                (test_forward_length < SOFT_UART_ISR_BUFFER_SIZE))
            {
                test_forward_buffer[test_forward_length++] = byte;
            }
        }

        /* 当前通道本周期收集到测试字节后才执行透传，空批次不占用 UART10。 */
        if (test_forward_length > 0U)
        {
            SoftUart_TestForwardFrame(channel, test_forward_buffer, test_forward_length);
        }
    }

    Cs1237_CheckPumpTimeout(); /* 两路字节搬运结束后统一检查有效帧超时，处理拔泵无数据的离线边沿。 */
}

/*
 * 函数功能：初始化两路模拟串口的 GPIO、独立采样定时器、接收缓冲和消息队列；重复调用不重新初始化。
 * 输入参数：无。
 * 返回参数：无。
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
    SoftUart_InitRxTimers();

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
        ctx->rx_stage = SOFT_UART_RX_STAGE_IDLE; /* 每路初始化为独立空闲态，允许两路同时捕获各自起始位。 */
        ctx->rx_bit_index = 0U;                  /* 清本通道位序号。 */
        ctx->rx_current_byte = 0U;               /* 清本通道临时字节。 */
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

    /* 两路起始位分别开放；后续每个字节结束也只重开自己的 EXTI，不再互相清 pending 位。 */
    SoftUart_EnableExti(SIM_UART_1);
    SoftUart_EnableExti(SIM_UART_2);
    /* 标记初始化完成 */
    s_initialized = 1U;
}

/* 创建后台搬运任务。
 * 对外暴露为单独初始化入口，便于在系统初始化阶段显式接入。 */
void SimUartTask_Init(void)
{
    SimUart_InitAll();

    /* 任务已经注册时不再创建第二份，避免两任务同时搬运同一环形缓冲。 */
    if (s_task_created)
    {
        return;
    }

    Kernel_TaskCreate(&sSimUartTaskHandle, SimUartTaskFunc);
    Kernel_TaskStart(&sSimUartTaskHandle, KERNEL_TASK_ALWAYS, 100U);
    s_task_created = 1U;
}

/*
 * 函数功能：通过指定模拟串口阻塞发送 1 字节；任一路正在接收时拒绝发送，防止破坏压力帧采样。
 * 输入参数：channel 为发送通道；data 为需要发送的字节。
 * 返回参数：发送成功返回 SOFT_UART_OK，通道无效返回 SOFT_UART_ERROR，接收忙返回 SOFT_UART_BUSY。
 */
SoftUART_Status SimUart_SendByte(sim_uart_channel_t channel, uint8_t data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);
    uint32_t primask;

    /* 通道号非法时没有可发送的 GPIO，返回错误且不关闭中断。 */
    if (ctx == NULL)
    {
        return SOFT_UART_ERROR;
    }

    SimUart_InitAll();

    if (SoftUart_AnyReceiverBusy())
    {
        return SOFT_UART_BUSY; /* 任一路正在接收时不执行关中断发送，避免破坏 9600bps 位采样。 */
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

    /* 进入发送前中断原本开启时才恢复，避免破坏调用方已有的临界区。 */
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
    /* 数据指针为空时拒绝批量发送，避免逐字节读取无效内存。 */
    if (data == NULL)
    {
        return SOFT_UART_ERROR;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        SoftUART_Status status = SimUart_SendByte(channel, data[i]);
        /* 任一字节发送失败时立即返回原错误码，后续字节不再继续破坏帧边界。 */
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
    /* 字符串指针为空时没有可发送内容，返回参数错误。 */
    if (str == NULL)
    {
        return SOFT_UART_ERROR;
    }

    while (*str != '\0')
    {
        SoftUART_Status status = SimUart_SendByte(channel, (uint8_t)*str);
        /* 任一字符发送失败时立即停止，避免输出半段后仍报告成功。 */
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

    /* 通道或输出缓存无效时不能访问队列，直接返回参数错误。 */
    if ((ctx == NULL) || (data == NULL))
    {
        return SOFT_UART_ERROR;
    }

    SimUart_InitAll();

    /* 兼容队列中有字节时返回成功；队列为空保持非阻塞并返回超时。 */
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

    /* 软件串口尚未初始化时没有有效队列，按可读字节数 0 返回。 */
    if (!s_initialized)
    {
        return 0U;
    }

    /* 非法通道没有队列，按可读字节数 0 返回以避免数组越界。 */
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

    /* 软件串口尚未初始化时没有可信统计值，保持调用方输出结构不变。 */
    if (!s_initialized)
    {
        return;
    }

    /* 通道或输出结构无效时拒绝快照，避免读取错误上下文或写空指针。 */
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

/*
 * 函数功能：处理 PE4/PE6 起始位下降沿，并启动该通道自己的半位采样定时器。
 * 输入参数：GPIO_Pin 为本次触发的 EXTI 引脚，PE4 对应 SIM_UART_1，PE6 对应 SIM_UART_2。
 * 返回参数：无。
 */
void SimUart_HandleExti(uint16_t GPIO_Pin)
{
    sim_uart_channel_t channel;
    SoftUartChannelContext *ctx;

    /* 软件串口未初始化时 GPIO 映射和定时器尚不可用，忽略外部中断。 */
    if (!s_initialized)
    {
        return;
    }

    /* PE4 起始位中断固定归属 A 压力通道，选择其独立接收状态机。 */
    if (GPIO_Pin == s_channels[SIM_UART_1].rx_pin)
    {
        channel = SIM_UART_1;
    }
    /* PE6 起始位中断固定归属 B 压力通道，选择另一套接收状态机。 */
    else if (GPIO_Pin == s_channels[SIM_UART_2].rx_pin)
    {
        channel = SIM_UART_2;
    }
    else
    {
        return;
    }

    ctx = SoftUart_GetChannel(channel);
    /* 未映射引脚或引脚已经回高都不是有效低电平起始位，不能启动采样定时器。 */
    if ((ctx == NULL) || (SoftUart_ReadRx(ctx) != GPIO_PIN_RESET))
    {
        return;
    }

    if (ctx->rx_stage == SOFT_UART_RX_STAGE_IDLE)
    {
        ctx->rx_stage = SOFT_UART_RX_STAGE_START; /* 只占用本通道接收状态，另一通道可同时进入 START。 */
        ctx->rx_bit_index = 0U;                   /* 新字节从 bit0 开始。 */
        ctx->rx_current_byte = 0U;                /* 新字节开始前清临时值。 */

        SoftUart_DisableExti(channel);                   /* 接收本字节期间关闭本通道 EXTI，避免数据位下降沿重复触发。 */
        SoftUart_StartSampleTimer(ctx, ctx->half_bit_time_us); /* 半位后确认本通道起始位仍为低。 */
    }
}

/*
 * 函数功能：处理指定通道的采样定时器更新中断，并推进该通道自己的位接收状态机。
 * 输入参数：channel 为定时器固定绑定的 SIM_UART_1 或 SIM_UART_2。
 * 返回参数：无。
 */
void SimUart_TimerIrqHandler(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx; /* 当前中断对应的固定通道上下文。 */

    /* 软件串口未初始化时定时器上下文无效，忽略共享中断入口。 */
    if (!s_initialized)
    {
        return;
    }

    ctx = SoftUart_GetChannel(channel);
    if (ctx == NULL)
    {
        return; /* 非法中断映射不访问定时器寄存器。 */
    }

    if ((ctx->rx_timer->DIER & TIM_DIER_UIE) == 0U)
    {
        return; /* 本通道更新中断未开放，忽略共享向量上的其它来源。 */
    }

    if ((ctx->rx_timer->SR & TIM_SR_UIF) == 0U)
    {
        return; /* 本通道没有更新标志，不推进状态机。 */
    }

    ctx->rx_timer->SR = 0U;          /* 只清当前通道定时器更新标志。 */
    SoftUart_ProcessSample(channel); /* 只推进当前通道，另一通道可在自己的 IRQ 中独立运行。 */
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
    /* 输出缓存为空时拒绝接收，避免轮询结果写入无效内存。 */
    if (data == NULL)
    {
        return SOFT_UART_ERROR;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        SoftUART_Status status = SimUart_ReadByte(SIM_UART_1, &data[i]);
        /* 当前字节未成功收到时按首字节超时和中途失败分别返回，避免误报完整帧。 */
        if (status != SOFT_UART_OK)
        {
            /* 第一个字节等待超时说明本轮完全无数据，向调用方保留明确超时状态。 */
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
