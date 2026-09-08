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

/* 中断先把字节放进小缓存，100ms 任务再解析并复制到队列。
 * 中断里不操作队列，以缩短中断执行时间；队列保留给需要逐字节读取的旧接口。 */
#define SOFT_UART_ISR_BUFFER_SIZE      64U /* 每路中断接收缓存容量，单位：字节；满后丢弃新字节并计数。 */
#define SOFT_UART_QUEUE_DEPTH          128U /* 每路旧接口可读取的字节队列容量；队列满不影响本任务已经完成的压力解析。 */
#define SOFT_UART_DEFAULT_BAUDRATE     9600U /* 两路压力串口波特率，单位：bit/s；必须与压力板一致，格式固定为 8N1。 */
#define SOFT_UART_IRQ_PRIORITY         4U  /* 数值越小优先级越高；4 高于 FreeRTOS 屏蔽阈值 5，使接收采样不被任务临界区延迟。 */
#define SOFT_UART_DIAG_LED             0 /* 发送指示灯开关：0 不操作 LED；1 在发送字节期间拉低 LED 引脚，结束后拉高。 */

/* 以下值来自压力上报协议，不是用户可调参数；修改必须同时核对压力板协议。 */
#define CS1237_FRAME_LENGTH            21U /* 一帧总长度，单位：字节；包括帧头、数据、CRC 和帧尾。 */
#define CS1237_HEADER_0                0xAAU /* 帧头第 1 字节，接收时先查找 AA 55。 */
#define CS1237_HEADER_1                0x55U /* 帧头第 2 字节。 */
#define CS1237_TAIL_0                  0x55U /* 帧尾第 1 字节，位于 frame[19]。 */
#define CS1237_TAIL_1                  0xAAU /* 帧尾第 2 字节，位于 frame[20]。 */
#define CS1237_PROTOCOL_VER            0x02U /* 只接受协议版本 2；其他版本不更新泵数据。 */
#define CS1237_MSG_TYPE_REPORT         0x01U /* 消息类型 1 表示压力板主动上报数据。 */
#define CS1237_PAYLOAD_LENGTH          0x0CU /* 数据区长度 12 字节；帧内长度字段必须与此值一致。 */
#define CS1237_CRC_OFFSET              17U /* CRC 低字节在 frame[17]、高字节在 frame[18]；数组下标从 0 开始。 */
#define CS1237_CRC_LENGTH              15U /* 从 frame[2] 起连续 15 字节参与 CRC，不包含帧头、CRC 本身和帧尾。 */
/* 根据压力板设备码识别泵类型；未列出的编码按未知泵处理，不允许启动。 */
#define CS1237_DEVICE_CODE_INJECT_WATER 0x07U  /* 压力板上报 0x07 时，主控识别为注水泵。 */
#define CS1237_DEVICE_CODE_POUR_WATER   0x0EU  /* 压力板上报 0x0E 时，主控识别为灌注泵。 */
#define CS1237_DEVICE_CODE_DRAW_WATER   0x0DU  /* 压力板上报 0x0D 时，主控识别为抽吸泵。 */
#define CS1237_PUMP_LOSS_SUSPECT_MS     1500U  /* 单位：ms；距上次有效帧超过 1500ms 时开始怀疑掉线，但暂不改变在线状态。 */
#define CS1237_PUMP_LOSS_CONFIRM_MS     3000U  /* 单位：ms；距上次有效帧超过 3000ms 才确认离线并清运行请求，不是疑似后再等 3 秒。 */
#define CS1237_RAW_MIN_VALUE            (-8388608L) /* 24 位有符号 ADC 原始计数的最小值，不是重量；低于此值的帧不使用。 */
#define CS1237_RAW_MAX_VALUE            8388607L /* 24 位有符号 ADC 原始计数的最大值；高于此值的帧不使用。 */
#define CS1237_WEIGHT_MAX_X10           500000U /* 接收重量上限，单位：0.1g，即 50000.0g；超过后不更新泵数据，不是报警阈值。 */
#define CS1237_THRESHOLD_MAX_G          50000U /* 压力板上报的阈值最大允许 50000g；用于拒绝异常数据，不是当前设定阈值。 */
#define CS1237_RECOVERY_VALID_FRAMES    3U /* 单位：帧；数值越界后需连续通过 3 次数值检查才恢复更新；新的越界帧会清零计数。 */

/* 位级接收状态机阶段定义。
 * 采用“起始位确认 -> 8 位数据 -> 停止位确认”的 8N1 接收流程。 */
typedef enum {
    SOFT_UART_RX_STAGE_IDLE = 0,
    SOFT_UART_RX_STAGE_START,
    SOFT_UART_RX_STAGE_DATA,
    SOFT_UART_RX_STAGE_STOP
} SoftUartRxStage;

/* 每路串口各自的引脚、定时器、接收进度和缓存。
 * A/B 同时上报时各收各的数据，不共用接收进度。 */
typedef struct {
    GPIO_TypeDef *tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef *rx_port;
    uint16_t rx_pin;
    uint32_t baudrate;                 /* 串口波特率，单位：bit/s，必须与压力板相同。 */
    uint16_t bit_time_us;              /* 一位数据占用的时间，单位：微秒。 */
    uint16_t half_bit_time_us;         /* 半位时间，单位：微秒；用于确认起始位是否保持低电平。 */

    TIM_TypeDef *rx_timer;             /* 本通道独占的位采样定时器；PE4 用 TIM11，PE6 用 TIM13。 */
    IRQn_Type rx_timer_irq;            /* 本通道采样定时器对应的 NVIC 中断号。 */
    uint8_t rx_timer_on_apb2;          /* 1 表示定时器在 APB2，0 表示在 APB1，用于计算真实 1MHz 预分频。 */
    volatile SoftUartRxStage rx_stage; /* 当前等待起始位、接收数据位，还是检查停止位。 */
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
static uint32_t s_cs1237_last_valid_tick[SIM_UART_COUNT]; /* 每路最近一次通过检查的压力帧时间，单位：ms；用于判断是否掉线。 */
static uint32_t s_cs1237_loss_suspect_tick[SIM_UART_COUNT]; /* 每路开始怀疑掉线的时间，单位：ms；0 表示当前没有等待确认。 */
static uint8_t s_cs1237_recovery_valid_count[SIM_UART_COUNT]; /* 数值越界后累计通过检查的帧数；再次越界就清零重数。 */
static bool s_cs1237_recovery_required[SIM_UART_COUNT]; /* true 表示该路出现过数值越界，暂时不接受单个正常帧恢复更新。 */

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

/*
 * 函数功能：检查通道号，并取得该路串口的引脚、缓存和接收状态。
 * 输入参数：channel 为 SIM_UART_1 或 SIM_UART_2。
 * 返回参数：该通道的数据结构指针；通道号无效时返回 NULL。
 */
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

    return false; /* 本次检查时两路都没有接收，允许发送函数继续处理。 */
}

/*
 * 函数功能：找到收到压力数据的通道所对应的 A 泵或 B 泵状态。
 * 输入参数：channel 为压力模块模拟串口通道，当前线束要求 SIM_UART_1/PE4 接 A 泵压力传感器，SIM_UART_2/PE6 接 B 泵压力传感器。
 * 返回参数：有效通道返回对应 pumpMessage_t 指针，非法通道返回 NULL。
 */
static pumpMessage_t *Cs1237_GetPump(sim_uart_channel_t channel)
{
    if (channel == SIM_UART_1)
    {
        return &pumpMessageA; /* PE4 收到的压力数据写入 A 泵；这里不改变泵驱动串口。 */
    }

    if (channel == SIM_UART_2)
    {
        return &pumpMessageB; /* PE6 收到的压力数据写入 B 泵；这里不改变泵驱动串口。 */
    }

    return NULL; /* 不是这两个压力通道就不选择任何泵。 */
}

/*
 * 函数功能：系统没有报警时，发出一次普通提示音。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Cs1237_BeepOnceIfNoAlarm(void)
{
    if (WorkMessage.alarm_flag == false)
    {
        SendKeyBeepMessage(1U); /* 普通提示音会清除蜂鸣任务的报警状态，所以有报警时不能用它提示泵接入或掉线。 */
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

/*
 * 函数功能：判断这一路压力字节是否需要额外发到 UART10 供测试查看。
 * 输入参数：channel 为字节所属通道。
 * 返回参数：配置选中这一路时返回 true，否则返回 false；默认关闭。
 */
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

/*
 * 函数功能：把选定压力通道的原始字节发到 UART10，不添加文字或换行。
 * 输入参数：channel 为压力通道；data 为原始字节；length 为字节数。
 * 返回参数：无；未选中该通道或数据为空时不发送。
 */
static void SoftUart_TestForwardFrame(sim_uart_channel_t channel, const uint8_t *data, uint16_t length)
{
    /* 没有数据就不占用测试串口。 */
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    /* 只输出选中的一路，避免 A/B 压力字节混在一起看不清。 */
    if (!SoftUart_IsForwardSource(channel))
    {
        return;
    }

    (void)Bsp_UartTransmit(BSP_UART_PORT_10,
                           (uint8_t *)data,
                           length,
                           SOFT_UART_TEST_FORWARD_UART_TIMEOUT_MS);
}

/*
 * 函数功能：检查发送时可用的微秒计时方式，优先用 DWT CPU 周期计数器，其次用已运行的 TIM7。
 * 输入参数：无。
 * 返回参数：无；结果保存在 s_dwt_ready 和 s_tim7_ready 中。TIM7 只用于发送延时，不参与接收采样。
 */
static void SoftUart_InitTimingBase(void)
{
    uint32_t start;

    /* 已有可用计数器就直接返回，不把正在使用的计数器清零。 */
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

    /* DWT 没有计数时，再检查 TIM7 是否正在计数；此处不重新配置 TIM7。 */
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
    /* DWT 可用时按 CPU 周期等待，使每一位保持指定的时间。 */
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
            /* 已等够指定时间就退出，继续发送下一位。 */
            if (elapsed >= remaining)
            {
                break;
            }
            remaining -= elapsed;
            start = now;
        }
        return;
    }

    /* 两个计数器都不可用时，先用 HAL 等待整毫秒部分，余下部分用空循环等待。 */
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
 * 输入参数：ctx 为该通道的配置；rx_timer_on_apb2 表示定时器接在 APB2 还是 APB1。
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
 * 函数功能：启动一次定时等待，到点后读取接收引脚；起始位等半位时间，之后每次等一位时间。
 * 输入参数：ctx 为当前通道数据；delay_us 为本次等待时间，单位：微秒。
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
 * 输入参数：ctx 为当前通道的定时器和接收状态。
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

/*
 * 函数功能：设置一路串口引脚；TX 默认输出高电平，RX 用下降沿中断检测起始位。
 * 输入参数：channel 为要初始化的模拟串口通道。
 * 返回参数：无；通道无效时不设置任何引脚。
 */
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

/*
 * 函数功能：在接收中断中保存 1 字节；缓存满时丢弃新字节并计数，不等待空位。
 * 输入参数：channel 为接收通道；data 为刚收到的字节。
 * 返回参数：无。
 */
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
 * 函数功能：由任务从中断接收缓存取出 1 字节，取出时暂时关闭中断。
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

    primask = __get_PRIMASK(); /* FreeRTOS 的 BASEPRI 不能挡住优先级 4 的接收中断，所以要保存全局中断状态后再关中断。 */
    __disable_irq(); /* 暂停所有中断，防止 TIM11/TIM13 ISR 写索引时任务同时修改同一通道计数。 */
    /* 有字节才读取并移动读位置；空缓存不改变读位置和数量。 */
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
 * 函数功能：定时器到点后读取本通道引脚电平，依次确认起始位、收 8 个数据位、检查停止位。
 * 输入参数：channel 为本次采样所属的模拟串口通道。
 * 返回参数：无。
 */
static void SoftUart_ProcessSample(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel); /* 由独立定时器中断明确选择通道。 */
    GPIO_PinState pin_state;

    /* 通道号无效时不读取引脚，也不改变接收进度。 */
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

    /* 版本、消息类型或数据区长度不匹配时，不把这 21 字节当作压力上报。 */
    if ((frame[2] != CS1237_PROTOCOL_VER) ||
        (frame[3] != CS1237_MSG_TYPE_REPORT) ||
        (frame[4] != CS1237_PAYLOAD_LENGTH))
    {
        return false; /* 格式不符合当前协议，不更新泵状态。 */
    }

    if ((frame[19] != CS1237_TAIL_0) || (frame[20] != CS1237_TAIL_1))
    {
        return false; /* 帧尾不对，不使用本帧，也不更新在线时间。 */
    }

    frame_crc = Cs1237_ReadU16Le(&frame[CS1237_CRC_OFFSET]);
    calc_crc = Cs1237_CalcCrc16Modbus(&frame[2], CS1237_CRC_LENGTH);

    return frame_crc == calc_crc; /* 收到的 CRC 必须与重新计算的结果一致，否则不使用本帧。 */
}

/*
 * 函数功能：检查压力帧中的数值是否越界；出现越界后，要连续通过 3 次数值检查才恢复更新。
 * 输入参数：channel 为A/B压力软串口通道；frame 指向已通过固定字段和CRC校验的21字节帧。
 * 返回参数：可以使用当前帧时返回 true；数值越界或正常帧数量还不够时返回 false。
 */
static bool Cs1237_BusinessDataTrusted(sim_uart_channel_t channel, const uint8_t *frame)
{
    uint32_t channel_index; /* 通道枚举直接对应A/B两路恢复状态数组下标。 */
    int32_t raw_cs1237; /* ADC 原始计数；必须落在 24 位有符号数范围内，不能出现异常大值。 */
    uint32_t weight_x10; /* 压力板上报重量，单位：0.1g，用于检查是否超过接收上限。 */
    uint16_t threshold_g; /* 压力板上报阈值，单位：g，用于检查是否超过接收上限。 */
    bool data_in_range; /* 三个数值都未越界，且设备码高 4 位为 0 时才为 true。 */

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
                    ((frame[16] & 0xF0U) == 0U); /* 设备码只使用 PA1~PA4 对应的低 4 位；高 4 位非零就拒绝本帧。 */

    if (data_in_range == false)
    {
        s_cs1237_recovery_required[channel_index] = true; /* 该路出现数值越界，后面只收到 1 个正常帧还不能恢复更新。 */
        s_cs1237_recovery_valid_count[channel_index] = 0U; /* 新异常会打断已有恢复计数，必须重新连续确认。 */
        return false; /* 不使用越界帧更新泵数据，在线超时计时也不会被它重置。 */
    }

    if (s_cs1237_recovery_required[channel_index] == false)
    {
        return true; /* 之前没有数值越界，本帧通过检查后可直接更新泵数据。 */
    }

    if (s_cs1237_recovery_valid_count[channel_index] < CS1237_RECOVERY_VALID_FRAMES)
    {
        s_cs1237_recovery_valid_count[channel_index]++; /* 每个连续合理帧只累计一次恢复确认。 */
    }
    if (s_cs1237_recovery_valid_count[channel_index] < CS1237_RECOVERY_VALID_FRAMES)
    {
        return false; /* 正常帧还不到 3 个，继续保留上次通过检查的压力值。 */
    }

    s_cs1237_recovery_required[channel_index] = false; /* 第 3 个正常帧到达，允许这一路重新更新泵数据。 */
    s_cs1237_recovery_valid_count[channel_index] = 0U; /* 恢复完成后清计数，下一次异常重新开始。 */
    return true; /* 当前第 3 帧就作为恢复后的第一帧使用。 */
}

/*
 * 函数功能：根据压力板上报的设备码，识别注水、灌注或抽吸泵。
 * 输入参数：device_code 为上报帧 frame[16] 的设备码，数组下标从 0 开始。
 * 返回参数：DRAWWATER/INJECTWATER/POURWATER 表示已识别泵类型，0 表示备用码或未知码。
 */
static uint16_t Cs1237_DecodePumpType(uint8_t device_code)
{
    /* 以压力板设备码决定泵类型，不能靠当前屏幕操作或手柄运行状态猜泵类型。 */
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
 * 函数功能：用通过检查的压力帧更新对应泵的数据；识别或掉线状态变化时刷新屏幕并提示。
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

    s_cs1237_last_valid_tick[(uint32_t)channel] = now_tick; /* 记下本次有效帧时间，之后从这里开始计算多久没收到有效数据。 */
    s_cs1237_loss_suspect_tick[(uint32_t)channel] = 0U; /* 已收到有效帧，取消之前的掉线怀疑。 */

    taskENTER_CRITICAL();
    old_pump_type = pump_message->type; /* 记下原泵类型，后面比较是否变化，避免每收到一帧都要求刷新屏幕。 */
    old_online_flag = pump_message->online_flag; /* 记录本帧前在线状态，未识别/重新识别时需要让屏幕可用状态同步变化。 */
    pump_message->pressure_value = raw_cs1237; /* 保存压力板的 ADC 原始计数，不是换算后的重量。 */
    pump_message->weight_x10 = weight_x10; /* 保存 0.1g 重量值，供屏幕或外部通信读取压力模块当前重量。 */
    pump_message->pressure_threshold = threshold_g; /* 保存压力板上报阈值，单位：g，供泵控制任务读取。 */
    /* 保存识别出的泵类型；泵任务据此选择方向和流量换算公式。 */
    pump_message->type = pump_type;
    pump_message->seq = frame[5]; /* 保存下位机帧序号，便于后续诊断压力模块是否连续上报。 */
    pump_message->online_flag = new_online_flag; /* 只有设备码映射到业务泵类型时才认为泵在线，备用码不允许启动泵。 */
    if (new_online_flag != false)
    {
        pump_message->losses_times = 0U; /* 已识别出支持的泵类型，清掉之前的识别丢失次数。 */
    }
    else
    {
        if (pump_message->losses_times < 0xFFU)
        {
            pump_message->losses_times++; /* 记录一次未识别；最多累计到 255，不再增加，避免回到 0。 */
        }
        pump_message->run_flag = false; /* 泵类型已经无效，强制停泵，避免沿用上一帧在线时的运行请求。 */
        pump_message->timingDrainage_flag = false; /* 清定时排空请求，避免重新识别后继承未知码阶段的排空状态。 */
        pump_message->speed_output = 0U; /* 清除记录的速度指令，屏幕和外部通信不再显示旧值；不是实测转速。 */
    }
    pump_online_changed = (old_online_flag != new_online_flag); /* 只在在线变离线、或离线变在线时提示一次，不每帧都响。 */
    pump_display_changed = ((old_pump_type != pump_type) ||
                            pump_online_changed); /* 泵类型或在线状态改变才刷新该泵区域，A 在左、B 在右。 */
    taskEXIT_CRITICAL();

    /* 只在泵类型或在线状态改变时设置默认流量/档位，不覆盖用户之后手动调整的值。 */
    if (pump_display_changed)
    {
        if(pump_message->type==INJECTWATER)
        {
        pump_message->speed_work = 30U; /* 注水泵刚识别时默认 30mL/min，之后仍可调速。 */
        }
        else if(pump_message->type==POURWATER)
        {
        pump_message->speed_work=200; /* 灌注泵刚识别时默认 200mL/min，之后仍可在屏幕调速。 */
        }
        else if(pump_message->type==DRAWWATER)
        {
        pump_message->speed_work=10; /* 抽水泵刚识别时装载 10 档默认设定，避免上线后保持零速。 */
        }
        Cs1237_RefreshPumpUi(channel); /* 把新的泵类型和在线状态显示到对应泵区。 */
    }

    if (pump_online_changed)
    {
        Cs1237_BeepOnceIfNoAlarm(); /* 在线状态刚改变时提示一次；有报警时不打断报警声。 */
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
            s_cs1237_loss_suspect_tick[i] = 0U; /* 距上次有效帧不超过 1500ms，取消之前的掉线怀疑。 */
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
            continue; /* 还没等到确认掉线的时间，暂时保留在线状态。 */
        }

        taskENTER_CRITICAL();
        if ((pump_message->online_flag != false) &&
            ((uint32_t)(now_tick - s_cs1237_last_valid_tick[i]) > CS1237_PUMP_LOSS_CONFIRM_MS))
        {
            pump_message->online_flag = false; /* 超时确认泵丢失，屏幕和外部通信后续都读到离线。 */
            pump_message->type = 0U; /* 清业务泵类型，避免屏幕或控制路径沿用旧类型继续允许启动。 */
            pump_message->run_flag = false; /* 泵丢失时强制退出运行，避免后续泵任务继续按旧状态输出。 */
            pump_message->timingDrainage_flag = false; /* 清定时排空状态，避免重新接入后继承拔泵前的排空请求。 */
            pump_message->speed_output = 0U; /* 清除记录的速度指令，屏幕和外部通信不再显示旧值。 */
            if (pump_message->losses_times < 0xFFU)
            {
                pump_message->losses_times++; /* 记录一次掉线；最多累计到 255，避免再加 1 后回到 0。 */
            }
            s_cs1237_loss_suspect_tick[i] = 0U; /* 已经确认离线并清业务状态，疑似阶段结束，等待下一次有效接入重新开始。 */
            loss_confirmed = true; /* 本周期刚从在线改为离线，后面需要刷新屏幕并提示一次。 */
        }
        taskEXIT_CRITICAL();

        if (loss_confirmed != false)
        {
            Cs1237_RefreshPumpUi(channel); /* 离线状态已经写入公共结构，立即把对应泵区刷为不可用。 */
            Cs1237_BeepOnceIfNoAlarm(); /* 掉线时提示一次；正在报警时不打断报警声。 */
        }
    }
}

/*
 * 函数功能：压力候选帧校验失败后，在现有缓存内寻找下一处 AA55 帧头并恢复解析位置。
 * 输入参数：parser 保存当前通道已收到的压力字节和长度。
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
 * 函数功能：逐字节收齐 21 字节压力帧，通过检查就更新泵数据；格式错误就重新寻找帧头。
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
        parser->length = 0U; /* 本帧已处理完，清长度并等待下一帧。 */
    }
    else
    {
        /* 当前 21 字节候选帧无效时，从缓冲内部继续寻找下一处 AA55，避免粘包错位后长期丢帧。 */
        Cs1237_ParserResync(parser);
    }
}

/*
 * 函数功能：每 100ms 取出两路接收字节、解析压力帧、复制到旧接口队列，并检查泵是否掉线。
 * 输入参数：event 为任务事件，本函数不使用。
 * 返回参数：无；串口每一位的电平采样由定时器中断完成，不在本任务里等待。
 */
static void SimUartTaskFunc(uint32_t event)
{
    uint8_t byte;
    uint8_t test_forward_buffer[SOFT_UART_ISR_BUFFER_SIZE];
    (void)event;

    if (!s_initialized)
    {
        return; /* 还没准备好接收缓存和队列，暂不取数据。 */
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
                ctx->queue_overflow_count++; /* 压力帧已先解析；旧接口队列满只记次数，不撤销刚更新的泵数据。 */
            }

            /* 开启测试后，只收集所选通道的原始字节，稍后统一发到 UART10。 */
            if (SoftUart_IsForwardSource(channel) &&
                (test_forward_length < SOFT_UART_ISR_BUFFER_SIZE))
            {
                test_forward_buffer[test_forward_length++] = byte;
            }
        }

        /* 本周期收到了测试数据才发送，没有数据就不占用 UART10。 */
        if (test_forward_length > 0U)
        {
            SoftUart_TestForwardFrame(channel, test_forward_buffer, test_forward_length);
        }
    }

    Cs1237_CheckPumpTimeout(); /* 处理完两路数据后，检查是否太久没收到有效压力帧，并处理刚发生的掉线。 */
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

    /* 准备发送用的微秒计时方式，以及接收用的 TIM11/TIM13。 */
    SoftUart_InitTimingBase();
    SoftUart_InitRxTimers();

    /* 遍历并初始化所有UART通道 */
    for (uint32_t i = 0U; i < SIM_UART_COUNT; i++)
    {
        /* 选中当前通道的配置和接收数据，A/B 分别初始化。 */
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

    /* 分别允许 PE4、PE6 检测起始位；每一路只清自己的待处理中断，不影响另一路。 */
    SoftUart_EnableExti(SIM_UART_1);
    SoftUart_EnableExti(SIM_UART_2);
    /* 标记初始化完成 */
    s_initialized = 1U;
}

/*
 * 函数功能：初始化两路压力串口，并创建每 100ms 执行一次的数据处理任务。
 * 输入参数：无。
 * 返回参数：无；重复调用不会重复创建任务。
 */
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

/*
 * 函数功能：按顺序发送缓存中的字节，任一字节发送失败就停止。
 * 输入参数：channel 为发送通道；data 为数据缓存；len 为发送字节数。
 * 返回参数：全部发完返回 SOFT_UART_OK；否则返回首次失败的状态，前面已发出的字节不会撤回。
 */
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
        /* 一个字节没发成功就停止，不跳过它继续发送后面的字节。 */
        if (status != SOFT_UART_OK)
        {
            return status;
        }
    }

    return SOFT_UART_OK;
}

/*
 * 函数功能：逐字符发送字符串，不发送结尾的 '\0'。
 * 输入参数：channel 为发送通道；str 为以 '\0' 结尾的字符串。
 * 返回参数：全部发完返回 SOFT_UART_OK；否则返回首次失败的状态。
 */
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

/*
 * 函数功能：从指定通道的字节队列取出 1 字节；没有数据时立即返回，不等待。
 * 输入参数：channel 为接收通道；data 用于保存取出的字节。
 * 返回参数：成功返回 SOFT_UART_OK，参数错误返回 SOFT_UART_ERROR，队列为空返回 SOFT_UART_TIMEOUT。
 */
SoftUART_Status SimUart_ReadByte(sim_uart_channel_t channel, uint8_t *data)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);

    /* 通道或输出缓存无效时不能访问队列，直接返回参数错误。 */
    if ((ctx == NULL) || (data == NULL))
    {
        return SOFT_UART_ERROR;
    }

    SimUart_InitAll();

    /* 队列里有字节就取出；等待时间设为 0，所以没有数据就立即返回。 */
    if (xQueueReceive(ctx->queue_handle, data, 0U) == pdPASS)
    {
        return SOFT_UART_OK;
    }

    return SOFT_UART_TIMEOUT;
}

/*
 * 函数功能：查看字节队列里还有多少数据可取，不移除数据。
 * 输入参数：channel 为要查询的通道。
 * 返回参数：待取字节数；尚未初始化或通道无效时返回 0，不包含中断缓存中的字节。
 */
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
 * 函数功能：一次复制指定通道的各项计数，复制时暂时关闭中断，避免读取中途数值变化。
 * 输入参数：channel 为模拟串口通道；stats 指向存放统计结果的结构体。
 * 返回参数：无。
 */
void SimUart_GetStats(sim_uart_channel_t channel, SimUartStats *stats)
{
    SoftUartChannelContext *ctx = SoftUart_GetChannel(channel);
    uint32_t primask; /* 保存原来的全局中断开关状态，复制结束后按原状态恢复。 */

    /* 软件串口尚未初始化时没有可信统计值，保持调用方输出结构不变。 */
    if (!s_initialized)
    {
        return;
    }

    /* 通道号或输出地址无效时不复制，保留调用方原有数据。 */
    if ((ctx == NULL) || (stats == NULL))
    {
        return;
    }

    primask = __get_PRIMASK(); /* 优先级 4 的接收中断不受 FreeRTOS 临界区限制，所以这里需要保存全局中断状态。 */
    __disable_irq(); /* 暂停 TIM11/TIM13/EXTI 中断，避免复制到一半时计数被更新。 */
    stats->received_bytes = ctx->received_bytes;
    stats->queue_overflow_count = ctx->queue_overflow_count;
    stats->buffer_overflow_count = ctx->buffer_overflow_count;
    stats->overlap_drop_count = ctx->overlap_drop_count;
    stats->framing_error_count = ctx->framing_error_count;
    if (primask == 0U)
    {
        __enable_irq(); /* 原来允许中断才重新打开；调用方原本关着中断时保持关闭。 */
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

    /* PE4 接 A 泵压力板，这次只处理通道 1。 */
    if (GPIO_Pin == s_channels[SIM_UART_1].rx_pin)
    {
        channel = SIM_UART_1;
    }
    /* PE6 接 B 泵压力板，这次只处理通道 2。 */
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
 * 函数功能：确认本通道定时器确实到点后，读取一次接收引脚并继续接收当前字节。
 * 输入参数：channel 为定时器固定绑定的 SIM_UART_1 或 SIM_UART_2。
 * 返回参数：无。
 */
void SimUart_TimerIrqHandler(sim_uart_channel_t channel)
{
    SoftUartChannelContext *ctx; /* 当前中断所对应的串口配置和接收进度。 */

    /* 软件串口还没初始化时，不处理这次定时器中断。 */
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
        return; /* 本通道没有开启定时中断，可能是共用中断入口的其他外设触发，直接返回。 */
    }

    if ((ctx->rx_timer->SR & TIM_SR_UIF) == 0U)
    {
        return; /* 本通道定时器没有到点，不改变接收进度。 */
    }

    ctx->rx_timer->SR = 0U;          /* 只清当前通道定时器更新标志。 */
    SoftUart_ProcessSample(channel); /* 只读取当前通道的一位，另一通道由自己的中断处理。 */
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

/*
 * 函数功能：通过旧接口从通道 1 连续取 len 字节；任一字节取不到就立即返回，不等待整帧。
 * 输入参数：data 为接收缓存；len 为希望取出的字节数。
 * 返回参数：全部取到返回 SOFT_UART_OK；否则返回失败状态，前面已取出的字节仍保留在 data 中。
 */
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
        /* 取不到当前字节就返回，不把已经取到的部分数据当作完整结果。 */
        if (status != SOFT_UART_OK)
        {
            /* 第一个字节就取不到时，本次未取出数据，返回队列为空的超时状态。 */
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
