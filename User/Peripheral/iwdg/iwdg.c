#include "iwdg.h"
#include "app_task.h"
#include <string.h>

#define IWDG_RECORD_MAGIC 0x57444731UL /* BKP0R为此值时，其后四个寄存器才是一份完整故障记录。 */
#define IWDG_THREAD_STACK_WORDS 256U /* 检查线程栈为256个32位字，即1KB；调小须重新检查栈余量。 */
#define IWDG_THREAD_PRIORITY (tskIDLE_PRIORITY + 4U) /* 比业务线程高一级，业务线程持锁卡住时仍能检查。 */

/* 编号从1起固定对应完整回调名；不能因某个初始化调用遗漏而自动免除该任务的检查。 */
static const char *const s_required_task_names[] =
{
    "Foot_ParseDataS",          /* 1：脚踏接收解析，未接脚踏时也应完成扫描。 */
    "FootControlTask",         /* 2：脚踏电机及泵动作处理。 */
    "HANDLESCANTaskFunc",      /* 3：手柄连接、认证和拔出检查。 */
    "HANDLEKEYTaskFunc",       /* 4：手柄实体键检查。 */
    "MOTORRUNTask",            /* 5：电机运行和停止命令发送。 */
    "MOTORUARTTaskFunc",       /* 6：电机反馈解析。 */
    "PUMPABehaviorTask",       /* 7：A泵控制及驱动状态处理。 */
    "PUMPBBehaviorTask",       /* 8：B泵控制及驱动状态处理。 */
    "SimUartTaskFunc",         /* 9：压力采集数据处理。 */
    "SCREENKEYTaskFunc",       /* 10：屏幕按键和触控保活解析。 */
    "KeyBehaviorsTask",        /* 11：脚踏、手柄和屏幕操作消息处理。 */
    "UIDISPLAYBehaviorTask",   /* 12：屏幕状态刷新。 */
    "AUTOMODEGETDATATaskFunc", /* 13：RFID请求推进，无标签也应返回。 */
    "ExternalCommTaskFunc"     /* 14：外控接收和断线处理，未连接也应返回。 */
};
#define IWDG_REQUIRED_TASK_COUNT (sizeof(s_required_task_names) / sizeof(s_required_task_names[0])) /* 必须运行的任务数量。 */

/* 各开机步骤允许的总时间，单位ms；同一步骤循环喂狗不能延长本步骤时间。 */
static const uint32_t s_boot_limits_ms[] =
{
    1000U, /* 系统时钟配置。 */
    1500U, /* 外设初始化。 */
    2000U, /* EEPROM初始化及版本同步。 */
    2000U, /* 业务串口配置与开机停止命令。 */
    8000U, /* 约4秒启动页，加上屏幕处理余量。 */
    2500U, /* 四次500ms固定等待。 */
    1500U, /* 单/双路RFID初始化。 */
    2000U  /* 业务任务和公共锁创建。 */
};

static task_t *s_required_tasks[IWDG_REQUIRED_TASK_COUNT]; /* 按固定编号保存任务对象，未创建项保持NULL。 */
static StaticTask_t s_iwdg_thread_tcb; /* 独立检查线程的静态管理区。 */
static StackType_t s_iwdg_thread_stack[IWDG_THREAD_STACK_WORDS]; /* 独立检查线程的静态栈。 */
static volatile IwdgStage_t s_iwdg_stage = IWDG_STAGE_CLOCK; /* 当前只允许该阶段指定的调用点喂狗。 */
static uint32_t s_stage_tick; /* 当前开机步骤或首轮等待开始时的HAL毫秒数。 */
static uint32_t s_last_feed_tick; /* 最近一次真正刷新硬件看门狗的HAL毫秒数。 */
static volatile uint32_t s_active_task_id; /* 当前持公共锁的关键任务编号，回调返回后清零。 */
static uint8_t s_backup_ready; /* 备份域可写时为1；早期异常不得访问尚未准备好的记录寄存器。 */
volatile IwdgResetRecord_t g_iwdg_last_reset; /* 本次启动保存的上次复位信息，供调试器查看。 */

/*
 * 函数功能：保存程序故障并立即请求主控复位；复位请求未完成时由已开启的IWDG继续计时。
 * 输入参数：reason为故障类别；task_id为超时或缺失任务编号，非任务故障填0。
 * 返回参数：无，不返回调用者；不等待业务锁、串口或EEPROM。
 */
void Iwdg_ForceReset(IwdgFault_t reason, uint32_t task_id)
{
    uint32_t failed_stage = (uint32_t)s_iwdg_stage; /* 先记住出错时的启动或运行阶段。 */
    __disable_irq(); /* 禁止中断和任务切换，不能在记录故障期间继续执行运行命令。 */
    s_iwdg_stage = IWDG_STAGE_FAULT; /* 从此不能通过任何业务入口喂狗。 */
    if (s_backup_ready != 0U)
    {
        RTC->BKP0R = 0U; /* 先撤销有效标记，写到一半的记录不能被当成完整记录。 */
        RTC->BKP1R = (uint32_t)reason; /* 保存软件发现的故障类别。 */
        RTC->BKP2R = task_id; /* 保存超时或缺失任务编号。 */
        RTC->BKP3R = (s_active_task_id & 0xFFU) | (failed_stage << 8U); /* 保存持锁任务及阶段，区分卡住者和等待者。 */
        RTC->BKP4R = ~(RTC->BKP1R ^ RTC->BKP2R ^ RTC->BKP3R); /* 校验记录字段，拒绝损坏记录。 */
        RTC->BKP0R = IWDG_RECORD_MAGIC; /* 字段写好后才发布完整记录。 */
    }
    __DSB(); /* 确保备份寄存器写入完成后再请求复位。 */
    NVIC_SystemReset(); /* 软件已经发现故障时直接复位，缩短等待硬件看门狗的时间。 */
    for (;;)
    {
        /* 复位请求尚未完成时保持停止执行，绝不能喂狗。 */
    }
}

/*
 * 函数功能：刷新硬件IWDG并保存喂狗时刻，调用者必须先通过本阶段的进度检查。
 * 输入参数：now为本次检查使用的HAL毫秒数。
 * 返回参数：无；刷新接口异常时复位。
 */
static void Iwdg_Refresh(uint32_t now)
{
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
        Iwdg_ForceReset(IWDG_FAULT_INIT, 0U); /* 刷新异常不能当作成功继续运行。 */
    }
    s_last_feed_tick = now; /* 只在真正刷新硬件后更新，调用检查函数本身不等于喂狗。 */
}

/*
 * 函数功能：尽早启用独立看门狗，并保存上次复位原因；后续时钟或初始化卡住也会复位。
 * 输入参数：无，必须在HAL_Init之后、SystemClock_Config之前调用一次。
 * 返回参数：无。
 */
void Iwdg_Init(void)
{
    g_iwdg_last_reset.reset_flags = RCC->CSR; /* 先保存硬件原因，不能先清标志。 */
    MX_IWDG_Init(); /* 按32kHz标称LSI约4秒，实际时间随芯片时钟偏差变化。 */
    __HAL_RCC_PWR_CLK_ENABLE(); /* 打开电源接口时钟，准备访问备份域。 */
    HAL_PWR_EnableBkUpAccess(); /* 允许写跨复位保留的寄存器，不把故障记录写到EEPROM。 */
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) == 0U)
    {
        __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI); /* 尚未选择RTC时钟时使用IWDG已开启的LSI。 */
    }
    __HAL_RCC_RTC_ENABLE(); /* 使备份寄存器所属RTC可访问，不配置日历或RTC中断。 */
    s_backup_ready = 1U; /* 从此软件故障可以保存一条复位记录。 */
    if (((g_iwdg_last_reset.reset_flags & RCC_CSR_PORRSTF) == 0U) &&
        (RTC->BKP0R == IWDG_RECORD_MAGIC) &&
        (RTC->BKP4R == ~(RTC->BKP1R ^ RTC->BKP2R ^ RTC->BKP3R)))
    {
        g_iwdg_last_reset.fault_reason = RTC->BKP1R; /* 只接收校验通过且不是上电遗留的记录。 */
        g_iwdg_last_reset.overdue_task = RTC->BKP2R; /* 保存软件发现的超时任务编号。 */
        g_iwdg_last_reset.active_task = RTC->BKP3R & 0xFFU; /* 保存当时正在执行的关键任务编号。 */
        g_iwdg_last_reset.stage = RTC->BKP3R >> 8U; /* 保存当时的开机、定标或运行阶段。 */
    }
    RTC->BKP0R = 0U; /* 记录已复制到RAM，下次硬件直接复位不能误用更早的故障。 */
    __HAL_RCC_CLEAR_RESET_FLAGS(); /* 读取完成后清旧标志，使下一次复位原因可区分。 */
    s_iwdg_stage = IWDG_STAGE_CLOCK; /* 新一轮启动从时钟配置开始。 */
    s_stage_tick = HAL_GetTick(); /* 记录第一个步骤的起点。 */
    s_last_feed_tick = s_stage_tick; /* HAL初始化IWDG时已经刷新硬件。 */
}

/*
 * 函数功能：判断本次是否由看门狗或软件复位启动，供各控制源要求松开后重新操作。
 * 输入参数：无。
 * 返回参数：1表示异常重启，0表示普通上电或外部复位。
 */
uint8_t Iwdg_WasAbnormalReset(void)
{
    return ((g_iwdg_last_reset.reset_flags & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF)) != 0U) ? 1U : 0U; /* 使用启动时保存的值，硬件标志已清除。 */
}

/*
 * 函数功能：核对开机顺序及上一步骤用时，再开始下一步骤。
 * 输入参数：stage必须是当前步骤紧接着的下一个开机步骤。
 * 返回参数：无；重复步骤、跨步骤或超时都会记录原因并复位。
 */
void Iwdg_BootStage(IwdgStage_t stage)
{
    uint32_t now = HAL_GetTick(); /* 顺序和超时检查使用同一时刻。 */
    if ((s_iwdg_stage >= IWDG_STAGE_TASKS) || (stage != (IwdgStage_t)(s_iwdg_stage + 1U)))
    {
        Iwdg_ForceReset(IWDG_FAULT_STAGE, 0U); /* 不能靠重复进入同一步骤延长喂狗时间。 */
    }
    if ((uint32_t)(now - s_stage_tick) >= s_boot_limits_ms[s_iwdg_stage])
    {
        Iwdg_ForceReset(IWDG_FAULT_BOOT_TIMEOUT, 0U); /* 阻塞调用最终返回也不能掩盖已发生的超时。 */
    }
    s_iwdg_stage = stage; /* 上一步完成后才接受下一步骤。 */
    s_stage_tick = now; /* 下一步骤独立计时。 */
    Iwdg_Refresh(now); /* 有限次数的实际步骤推进允许刷新硬件。 */
}

/*
 * 函数功能：启动页完成一轮处理后检查总时限，再按100ms间隔喂狗。
 * 输入参数：无，只允许在启动页阶段调用。
 * 返回参数：无；阶段错误或启动页超过8秒会复位。
 */
void Iwdg_BootPoll(void)
{
    uint32_t now = HAL_GetTick(); /* 毫秒时钟停止时，下面的间隔条件不会再次允许喂狗。 */
    if (s_iwdg_stage != IWDG_STAGE_STARTUP_PAGE)
    {
        Iwdg_ForceReset(IWDG_FAULT_STAGE, 0U); /* 运行任务不能借启动入口绕过业务检查。 */
    }
    if ((uint32_t)(now - s_stage_tick) >= s_boot_limits_ms[IWDG_STAGE_STARTUP_PAGE])
    {
        Iwdg_ForceReset(IWDG_FAULT_BOOT_TIMEOUT, 0U); /* 循环喂狗不能延长启动页8秒总时限。 */
    }
    if ((uint32_t)(now - s_last_feed_tick) >= IWDG_CHECK_PERIOD_MS)
    {
        Iwdg_Refresh(now); /* 只有完整循环返回且时钟继续前进，才刷新硬件。 */
    }
}

/*
 * 函数功能：从启动页进入独立脚踏定标检查，允许长期停留但不允许定标程序卡住。
 * 输入参数：无。
 * 返回参数：无；其它阶段调用会复位。
 */
void Iwdg_EnterCalibration(void)
{
    Iwdg_BootPoll(); /* 先确认启动页没有超时，不能借定标绕过启动故障。 */
    s_iwdg_stage = IWDG_STAGE_CALIBRATION; /* 正常业务任务尚未创建，改为检查完整定标循环。 */
    Iwdg_Refresh(HAL_GetTick()); /* 明确交接到定标阶段后刷新一次硬件。 */
}

/*
 * 函数功能：屏幕、脚踏解析及显示处理全部返回后，检查定标循环用时并喂狗。
 * 输入参数：无。
 * 返回参数：无；阶段错误或定标处理超过允许间隔会复位。
 */
void Iwdg_CalibrationCycleComplete(void)
{
    uint32_t now = HAL_GetTick(); /* 只在整轮处理完成后读取时间。 */
    if (s_iwdg_stage != IWDG_STAGE_CALIBRATION)
    {
        Iwdg_ForceReset(IWDG_FAULT_STAGE, 0U); /* 正常业务不能使用定标入口单独喂狗。 */
    }
    if ((uint32_t)(now - s_last_feed_tick) >= IWDG_TASK_TIMEOUT_MS)
    {
        Iwdg_ForceReset(IWDG_FAULT_TASK_TIMEOUT, 0U); /* 超过1秒才返回也不能补喂掩盖阻塞。 */
    }
    if ((uint32_t)(now - s_last_feed_tick) >= IWDG_CHECK_PERIOD_MS)
    {
        Iwdg_Refresh(now); /* 无按键或无脚踏数据也可以正常完成处理，不要求外设在线。 */
    }
}

/*
 * 函数功能：按完整回调名登记关键任务，不依赖RTOS截断后的线程名。
 * 输入参数：task为刚创建的静态业务任务对象。
 * 返回参数：1到14为关键任务编号，0为非关键任务。
 */
uint8_t Iwdg_RegisterTask(struct task_s *task)
{
    uint32_t i; /* 固定表下标，不按已经创建的数量缩小检查范围。 */
    if ((task == NULL) || (task->name == NULL))
    {
        return 0U; /* 无名称对象不能顶替必须运行的任务。 */
    }
    for (i = 0U; i < IWDG_REQUIRED_TASK_COUNT; ++i)
    {
        if (strcmp(task->name, s_required_task_names[i]) == 0)
        {
            if (s_required_tasks[i] != NULL)
            {
                Iwdg_ForceReset(IWDG_FAULT_TASK_MISSING, i + 1U); /* 禁止同名重复登记掩盖其中一个卡住任务。 */
            }
            s_required_tasks[i] = task; /* 保存稳定的静态任务地址，运行期只读取。 */
            return (uint8_t)(i + 1U); /* 0保留给非关键任务。 */
        }
    }
    return 0U; /* 非关键任务仍正常调度，持锁卡住时由关键任务超时发现。 */
}

/*
 * 函数功能：回调真正开始前记录任务编号，故障时可确定哪个关键任务持有公共锁。
 * 输入参数：task_id为任务登记编号，非关键任务为0。
 * 返回参数：无。
 */
void Iwdg_TaskEnter(uint8_t task_id)
{
    s_active_task_id = task_id; /* 只记开始，不能更新完成时刻或喂狗。 */
}

/*
 * 函数功能：业务回调正常返回后更新完成记录，供独立线程检查实际进度。
 * 输入参数：task为刚执行完回调的静态任务对象。
 * 返回参数：无。
 */
void Iwdg_TaskComplete(struct task_s *task)
{
    uint32_t now = HAL_GetTick(); /* 本轮真正返回时再检查，避免刚好赶在检查线程前补写记录掩盖超时。 */
    if ((s_iwdg_stage == IWDG_STAGE_RUNNING) && (task->watchdog_id != 0U) &&
        task->watchdog_completed && ((uint32_t)(now - task->watchdog_tick) >= IWDG_TASK_TIMEOUT_MS))
    {
        Iwdg_ForceReset(IWDG_FAULT_TASK_TIMEOUT, task->watchdog_id); /* 已经超过允许完成间隔，不能补写新时刻继续运行。 */
    }
    taskENTER_CRITICAL(); /* 时刻和首次完成标志一起更新，防止检查线程读到只写一半的数据。 */
    task->watchdog_tick = now; /* 等待锁或进入回调不算完成。 */
    task->watchdog_completed = true; /* 第一轮完整返回后才标记已启动。 */
    s_active_task_id = 0U; /* 当前回调已返回，后续故障不能误指向刚完成的任务。 */
    taskEXIT_CRITICAL(); /* 完成记录写好后恢复调度。 */
}

/*
 * 函数功能：检查所有关键任务是否仍被启动且按时完成，再决定是否喂狗。
 * 输入参数：无。
 * 返回参数：无；缺失、停止或超时任务都会记录原因并复位。
 */
static void Iwdg_CheckRuntime(void)
{
    uint32_t now = HAL_GetTick(); /* 同一轮检查使用同一时刻，无符号相减允许计数回绕。 */
    uint32_t i; /* 固定任务表下标。 */
    uint32_t waiting_task = 0U; /* 首轮未完成任务编号，0表示全部完成。 */
    if ((s_iwdg_stage != IWDG_STAGE_WAIT_TASKS) && (s_iwdg_stage != IWDG_STAGE_RUNNING))
    {
        Iwdg_ForceReset(IWDG_FAULT_STAGE, 0U); /* 只有正常调度阶段允许检查线程喂狗。 */
    }
    for (i = 0U; i < IWDG_REQUIRED_TASK_COUNT; ++i)
    {
        task_t *task = s_required_tasks[i]; /* 逐项检查，不能只统计还有多少任务在运行。 */
        if ((task == NULL) || (!task->threadCreated) || (task->threadHandle == NULL) ||
            (!task->start) || task->oneShot || (task->period == 0U))
        {
            Iwdg_ForceReset(IWDG_FAULT_TASK_MISSING, i + 1U); /* 关键任务被停止后仍须检查，不能自动豁免。 */
        }
        if (!task->watchdog_completed)
        {
            waiting_task = i + 1U; /* 只允许在有限的首轮等待中暂时没有完成记录。 */
        }
        else if ((uint32_t)(now - task->watchdog_tick) >= IWDG_TASK_TIMEOUT_MS)
        {
            Iwdg_ForceReset(IWDG_FAULT_TASK_TIMEOUT, i + 1U); /* 覆盖内部卡死、等锁和未被调度。 */
        }
    }
    if (s_iwdg_stage == IWDG_STAGE_WAIT_TASKS)
    {
        if (waiting_task == 0U)
        {
            s_iwdg_stage = IWDG_STAGE_RUNNING; /* 全部完成过才结束首轮等待，下一轮开始检查最近完成时刻。 */
        }
        else if ((uint32_t)(now - s_stage_tick) >= IWDG_TASK_START_TIMEOUT_MS)
        {
            Iwdg_ForceReset(IWDG_FAULT_TASK_TIMEOUT, waiting_task); /* 其它任务正常不能无限延长首轮等待。 */
        }
    }
    else if (waiting_task != 0U)
    {
        Iwdg_ForceReset(IWDG_FAULT_TASK_MISSING, waiting_task); /* 运行后首次完成标志异常丢失也必须处理。 */
    }
    if ((uint32_t)(now - s_last_feed_tick) >= IWDG_CHECK_PERIOD_MS)
    {
        Iwdg_Refresh(now); /* 检查通过或仍在有限首轮等待内，才允许刷新硬件。 */
    }
}

/*
 * 函数功能：每100ms检查业务进度，不获取公共业务锁，也不调用可能阻塞的通信。
 * 输入参数：argument为RTOS线程参数，当前不使用。
 * 返回参数：无，线程常驻运行。
 */
static void Iwdg_Thread(void *argument)
{
    (void)argument; /* 检查对象来自启动时登记的固定任务表。 */
    for (;;)
    {
        Iwdg_CheckRuntime(); /* 正常运行阶段只由此线程喂狗。 */
        vTaskDelay(pdMS_TO_TICKS(IWDG_CHECK_PERIOD_MS)); /* 主动让出CPU，不能占住较高优先级忙等。 */
    }
}

/*
 * 函数功能：业务任务和公共锁创建后，创建独立检查线程并开始有限首轮等待。
 * 输入参数：无，必须在MX_FREERTOS_Init后、vTaskStartScheduler前调用一次。
 * 返回参数：无；创建失败会记录原因并复位。
 */
void IwdgTaskInit(void)
{
    uint32_t now = HAL_GetTick(); /* 任务创建阶段结束时刻。 */
    if ((s_iwdg_stage != IWDG_STAGE_TASKS) ||
        ((uint32_t)(now - s_stage_tick) >= s_boot_limits_ms[IWDG_STAGE_TASKS]))
    {
        Iwdg_ForceReset(IWDG_FAULT_BOOT_TIMEOUT, 0U); /* 顺序错误或创建超时都不能转入正常喂狗。 */
    }
    if ((AppTaskScheduler_CreateResult() != (int32_t)pdPASS) ||
        (xTaskCreateStatic(Iwdg_Thread, "IwdgSupervisor", IWDG_THREAD_STACK_WORDS, NULL,
                          IWDG_THREAD_PRIORITY, s_iwdg_thread_stack, &s_iwdg_thread_tcb) == NULL))
    {
        Iwdg_ForceReset(IWDG_FAULT_INIT, 0U); /* 没有检查线程时不能继续业务运行。 */
    }
    s_iwdg_stage = IWDG_STAGE_WAIT_TASKS; /* 进入有总时限的等待，不能永久忽略缺失任务。 */
    s_stage_tick = now; /* 首轮等待最多3秒，任务完成不能延长总时间。 */
    Iwdg_Refresh(now); /* 启动调度器前完成一次受阶段限制的喂狗交接。 */
}
