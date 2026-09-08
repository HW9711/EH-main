//screenkey.c

#include "screenkey.h"
#include "screen_address.h"  /* 读取泵显示镜像宏，保证触摸键区和显示位置同向交换。 */
#include "uart6.h"
#include "data.h"
#include "common.h"
#include "Pubinterface.h"
#include "sscKEYBH.h"
#include "sscBEEP.h"

#include "kernel_scheduler.h"

kernel_task_t SCREENKEYTaskHandle;

/* 启动页和脚踏定标页仍沿用少量旧按键编码，这里只保存一次性事件，不再回写 旧全局键值。 */
static uint8_t s_screenkey_legacy_event = KEY_NONE;
/* 报警后确认松手所需的无信号周期数：14 × 30ms，约 420ms。增大会延长再次按下前的等待，减小会更容易把短暂断包当成松手。 */
#define SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS 14U
/* 0x5520 是屏幕持续发送的“仍在按住”信号。只记录成功放入按键队列的时刻，队列满而丢弃的信号不能延长运行时间。 */
static volatile uint32_t s_touch_keepalive_accepted_tick_ms = 0U;
/* 1 表示本次触控已有有效的按住信号；退出触控、进入外控或报警时清零。 */
static volatile uint8_t s_touch_keepalive_accepted_valid = 0U;
/* 触控长按过程中发生报警后置位，必须等屏幕停止发送 0x5520 一段时间才允许再次运行。 */
static uint8_t s_touch_alarm_release_required = 0U;
/* 报警后用于判断是否松手：收到 0x5520 就清零，一直收不到才逐渐数到松手确认值。 */
static uint8_t s_touch_alarm_release_ticks = SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS;

/*
 * 函数功能：保存启动页或脚踏定标页的一次性旧按键事件。
 * 输入参数：key_value 为旧页面按键编号；KEY_NONE 表示没有事件。
 * 返回参数：无。
 */
void ScreenKey_LegacyEventPost(uint8_t key_value)
{
  /* KEY_NONE 表示没有事件；非空键值只保留最后一次，行为与旧全局键值被覆盖的方式一致。 */
  if (key_value != KEY_NONE) /* 没有按键时不能覆盖尚未处理的有效按键。 */
  {
    s_screenkey_legacy_event = key_value;
  }
}

/*
 * 函数功能：取出并清除启动页或脚踏定标页的一次性旧按键事件。
 * 输入参数：无。
 * 返回参数：返回最近一次事件；没有事件时返回 KEY_NONE。
 */
uint8_t ScreenKey_LegacyEventTake(void)
{
  uint8_t key_value = s_screenkey_legacy_event;

  /* 取出后立即清空，保证启动页和定标页不会重复处理同一次按键。 */
  s_screenkey_legacy_event = KEY_NONE;

  return key_value;
}

/*
 * 函数功能：取消之前收到的按住信号，后续不能再靠旧信号启动或维持电机运行。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_InvalidateTouchKeepAlive(void)
{
  s_touch_keepalive_accepted_valid = 0U; /* 先撤销有效标志，退出触控、外控或报警后旧保活不得再次启动。 */
  __DMB(); /* 保证上面的清零先完成，其他任务不能误用旧的有效标志。 */
}

/*
 * 函数功能：按住信号已成功放入按键队列后，记录它的接收时刻。
 * 输入参数：无，函数在SendKeyBehMessage确认成功后读取HAL毫秒时钟。
 * 返回参数：无。
 */
static void ScreenKey_RecordAcceptedTouchKeepAlive(void)
{
  s_touch_keepalive_accepted_tick_ms = HAL_GetTick(); /* 从成功放入队列时起计算 420ms；未放入队列的信号不算。 */
  __DMB(); /* 先写好接收时刻，再允许其他任务按“有效信号”读取。 */
  s_touch_keepalive_accepted_valid = 1U; /* 标记本次触控已收到有效的按住信号。 */
}

/*
 * 函数功能：检查最近一次已放入按键队列的按住信号是否还有效。
 * 输入参数：无，读取当前HAL毫秒时钟和最近接受时刻。
 * 返回参数：1 表示未到 420ms；0 表示没有有效信号、信号已取消或已到 420ms。
 */
uint8_t ScreenKey_IsAcceptedTouchKeepAliveFresh(void)
{
  uint32_t accepted_tick_ms; /* 只取一次接收时刻，避免计算期间被新信号更新。 */

  if (s_touch_keepalive_accepted_valid == 0U)
  {
    return 0U; /* 本次触控没有有效的按住信号，不能继续运行。 */
  }

  __DMB(); /* 先确认有效标志，再读取对应的接收时刻。 */
  accepted_tick_ms = s_touch_keepalive_accepted_tick_ms; /* 保存接收毫秒数；用无符号数相减，可处理时钟计满后从零开始的情况。 */
  return (((uint32_t)(HAL_GetTick() - accepted_tick_ms) < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_MS) ? 1U : 0U); /* 到达420ms边界即判过期。 */
}

/*
 * 函数功能：报警后仍收到按住信号时，重新开始等待松手。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_MarkAlarmKeepAlive(void)
{
  s_touch_alarm_release_ticks = 0U; /* 原始 0x5520 仍在持续发送，说明用户还没有松开触控按钮。 */
}

/*
 * 函数功能：报警中或报警后尚未松手时，拦住本次 0x5520 按住信号。
 * 输入参数：无。
 * 返回参数：true 表示本帧不投递业务队列；false 表示允许按普通触控保活处理。
 */
static uint8_t ScreenKey_BlockAlarmKeepAlive(void)
{
  if (WorkMessage.alarm_flag == true) /* 当前存在真实报警时，持续触摸不能继续维持电机运行。 */
  {
    ScreenKey_InvalidateTouchKeepAlive(); /* 报警建立后撤销报警前已接受的保活，防止排队旧事件在报警解除后续跑。 */
    s_touch_alarm_release_required = 1U; /* 长按运行过程中出现真实报警后进入“必须松手”状态。 */
    ScreenKey_MarkAlarmKeepAlive(); /* 报警期间收到的本帧只能证明仍在按压，不能继续运行。 */
    return 1U; /* 报警帧不再投递到 ControlTypeActive，避免报警解除后同一次长按继续启动。 */
  }

  if (s_touch_alarm_release_required != 0U) /* 报警已解除但还没松手，仍不允许恢复运行。 */
  {
    ScreenKey_MarkAlarmKeepAlive(); /* 报警已解除但原始保活仍在，继续等待用户松手。 */
    return 1U; /* 松手确认完成前，不把按住信号交给运行控制。 */
  }

  return 0U; /* 没有报警且不再等待松手，可正常处理按住信号。 */
}

/*
 * 函数功能：周期检查 8 寸屏触控保活是否超时。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_ServiceKeepAlive(void)
{

  if (WorkMessage.hmiactive_work != 0U) /* 外控复用 TOUCHWORK，但不属于屏幕触控保活。 */
  {
    ScreenKey_InvalidateTouchKeepAlive(); /* 外控运行时取消旧触控信号，退出外控后不能靠旧信号启动。 */
    s_touch_alarm_release_required = 0U; /* 外控期间不等待触控松手，下次触控重新判断。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS; /* 同步复位释放计数，避免外控结束后沿用旧触控长按状态。 */
    return; /* 外控也使用 TOUCHWORK 标志，但不能被屏幕按住信号超时而停机。 */
  }

  if ((WorkMessage.drivetype_work != TOUCHWORK) || (WorkMessage.touchactive_work != TOUCHWORK)) /* 只有本机触控已激活才进入保活状态机。 */
  {
    ScreenKey_InvalidateTouchKeepAlive(); /* 不在触控模式就取消旧信号，下次触控必须重新收到有效信号。 */
    s_touch_alarm_release_required = 0U; /* 退出触控后清除“报警后先松手”的要求，下次触控重新判断。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS; /* 同步恢复释放计数到空闲态。 */
    return; /* 非触控控制源不能被本任务的保活超时逻辑停止。 */
  }

  if (WorkMessage.alarm_flag == true) /* 报警出现后立即取消旧信号，不必等下一次 0x5520 才处理。 */
  {
    ScreenKey_InvalidateTouchKeepAlive(); /* 旧保活消息即使尚在按键队列中，也不能在报警解除后恢复本次运行。 */
  }


  if (s_touch_alarm_release_required != 0U) /* 报警后必须检测到一段无保活时间，才认定用户已经松手。 */
  {
    if (s_touch_alarm_release_ticks < SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS) /* 未到松手确认时间时继续累计空闲周期。 */
    {
      s_touch_alarm_release_ticks++; /* 每次检查加一；若仍收到按住信号，接收函数会把此计数清零。 */
    }
    if (s_touch_alarm_release_ticks >= SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS) /* 连续14个扫描周期未收到保活，确认本次按压已经释放。 */
    {
      s_touch_alarm_release_required = 0U; /* 原始保活帧已经停止约420ms，确认用户松手，可允许下一次按压。 */
      Pubinterface_ReleaseTouchHandleNotConnectedAlarm(); /* 若本次松手对应运行中拔手柄报警，则退出触控控制源并关闭报警弹窗。 */
    }
  }

  if ((WorkMessage.runflag_work == true) && (ScreenKey_IsAcceptedTouchKeepAliveFresh() == 0U)) /* 触控运行时必须持续收到有效的按住信号。 */
  {
    ScreenKey_InvalidateTouchKeepAlive(); /* 超时就取消旧信号，下次启动必须重新收到有效信号。 */
    s_touch_alarm_release_required = 0U; /* 普通松手停机不需要再等待一次“报警后松手”。 */
    Pubinterface_StopTouchKeepAliveRun(); /* 420ms内没有成功入队新保活时只停电机，不退出触控界面。 */
  }
}

/*
 * 函数功能：把屏幕按键编号转换为当前业务按键，确认此键允许使用后再放入按键队列。
 * 输入参数：legacy_key为屏幕串口协议解析出的旧页面按键编号。
 * 返回参数：无；未定义、不可用或队列已满的按键不执行、不蜂鸣，也不延长触控运行时间。
 * 屏幕仍发送原编号；此处转成 SCREENKey_* 后由 sscKEYBH 调用对应业务函数。
 */
static void ScreenKey_PostLegacyAction(uint8_t legacy_key)
{
  uint8_t screen_key = 0U; /* 0 表示旧编号没有现行业务动作，映射失败时必须保持静默。 */

  switch (legacy_key)
  {
    case 1U:
    case 2U:
      screen_key = SCREENKey_SPEED_Add;
      break;

    case 3U:
    case 4U:
      screen_key = SCREENKey_SPEED_Sub;
      break;

    case 5U:
      screen_key = SCREENKey_BPUMP_Add;
      break;

    case 6U:
      screen_key = SCREENKey_BPUMP_Sub;
      break;

    case 7U:
      screen_key = SCREENKey_APUMP_Add;
      break;

    case 8U:
      screen_key = SCREENKey_APUMP_Sub;
      break;

    case 9U:
      screen_key = SCREENKey_FREQ_Sub;
      break;

    case 10U:
      screen_key = SCREENKey_FREQ_Add;
      break;

    case 11U:
      screen_key = SCREENKey_BPUMP_control;
      break;

    case 12U:
      screen_key = SCREENKey_APUMP_control;
      break;

    case 13U:
      screen_key = SCREENKey_Dir_Forward;
      break;

    case 14U:
      screen_key = SCREENKey_Dir_Reverse;
      break;

    case 15U:
      screen_key = SCREENKey_Dir_OSC;
      break;

    case 16U:
      screen_key = SCREENKey_JTActi;
      break;

    case 17U:
      screen_key = SCREENKey_HandleActi;
      break;

    case 18U:
      screen_key = SCREENKey_TouchActi;
      break;

    case 20U:
      screen_key = SCREENKey_GrindH;
      break;

    case 21U:
      screen_key = SCREENKey_PlanerH;
      break;

    case 22U:
      screen_key = SCREENKey_OpenPos_ClockWise;
      break;

    case 23U:
      screen_key = SCREENKey_OpenPos_AntiClockWise;
      break;

    case 24U:
      screen_key = SCREENKey_HANDLE_A;
      break;

    case 25U:
      screen_key = SCREENKey_HANDLE_B;
      break;

    case 26U:
      screen_key = SCREENKey_UNPLUG_A;
      break;

    case 27U:
      screen_key = SCREENKey_UNPLUG_B;
      break;

    case 28U:
      screen_key = SCREENKey_PLUG_A;
      break;

    case 29U:
      screen_key = SCREENKey_PLUG_B;
      break;

    case 40U:
      break; /* 40 号旧触控退出入口已废弃，保持静默，防止重复退出事件。 */

    case 42U:
      screen_key = SCREENKey_TouchEXIT;
      break;

    case 43U:
      screen_key = SCREENKey_HMI_EXIT;
      break;

    case 44U: /* 0x5520 按住信号在本函数中使用编号 44。 */
      screen_key = SCREENKey_TouchKeepAlive; /* 8 寸屏 0x5520 触控按住保活，持续收到才允许触控运行。 */
      break;

    case 30U:
      screen_key = SCREENKey_SPEED_Sub_Large; /* 新屏速度快减键，业务层按当前方向步进的两倍减少。 */
      break;

    case 31U:
      screen_key = SCREENKey_SPEED_Sub_Small; /* 新屏速度慢减键，业务层按当前方向寄存器步进减少。 */
      break;

    case 32U:
      screen_key = SCREENKey_SPEED_Add_Small; /* 新屏速度慢加键，业务层按当前方向寄存器步进增加。 */
      break;

    case 33U:
      screen_key = SCREENKey_SPEED_Add_Large; /* 新屏速度快加键，业务层按当前方向步进的两倍增加。 */
      break;

    case 36U:
      screen_key = SCREENKey_AutoIdentify; /* 新屏自动识别键沿用旧 36 号入口，但业务层改为明确 RFID 事件。 */
      break;

    case 50U:
      screen_key = SCREENKey_HMI_EXIT; /* 新屏幕资源的强制退出按钮复用外控退出行为，只补入口不改业务仲裁。 */
      break;

    default:
      break;
  }

  if (screen_key != 0U) /* 只有成功映射为现行业务键的触摸才进入可用性判断，未定义坐标保持静默。 */
  {
    uint8_t beep_enable = 1U; /* 默认所有有效触控按键响一声，给操作者明确反馈。 */
    if (screen_key == SCREENKey_TouchKeepAlive) /* 处理按住信号前，先确认报警后已松手。 */
    {
      if (ScreenKey_BlockAlarmKeepAlive() != 0U) /* 报警期间或报警后仍在长按时，当前保活帧必须静默丢弃。 */
      {
        return; /* 报警后仍在长按时不执行、不蜂鸣；必须松手后重新按下。 */
      }
    }

    if (ScreenKey_CanUse(screen_key) == false) /* 当前图标必须处于可操作状态，黑色或隐藏图片不接受触摸。 */
    {
      return; /* 黑色、隐藏或当前业务状态不可用的坐标静默丢弃，既不蜂鸣也不进入业务队列。 */
    }

    if (screen_key == SCREENKey_TouchKeepAlive) /* 只有触控运行保活需要维护连续帧超时和重复蜂鸣。 */
    {
      if (ScreenKey_IsAcceptedTouchKeepAliveFresh() != 0U) /* 上一份成功入队保活仍在420ms窗口内时视为同一次长按。 */
      {
        beep_enable = 0U; /* 0x5520 连续保活帧不重复蜂鸣，只在刚按下或超时后重新按下时响一次。 */
      }
    }

    if (SendKeyBehMessage(SCREENKey, screen_key) == false) /* 只有消息真正进入统一行为队列才允许改变保活和蜂鸣状态。 */
    {
      return; /* 队列满或尚未创建时不延长有效时间，也不蜂鸣，避免提示已接受实际却未处理。 */
    }

    if (screen_key == SCREENKey_TouchKeepAlive) /* 成功放入队列后才更新本次按住信号的有效时间。 */
    {
      ScreenKey_RecordAcceptedTouchKeepAlive(); /* 业务队列已经持有该事件，此时续期不会掩盖投递失败。 */
    }
    else if (screen_key == SCREENKey_TouchEXIT) /* 用户主动退出触控时，立即结束报警后的松手等待。 */
    {
      ScreenKey_InvalidateTouchKeepAlive(); /* 退出事件已成功排队，当前会话旧保活从此不得延迟启动。 */
      s_touch_alarm_release_required = 0U; /* 主动退出触控后，不再等待本次触控松手。 */
      s_touch_alarm_release_ticks = SCREENKEY_TOUCH_RELEASE_TIMEOUT_TICKS; /* 下一次进入触控重新计算报警后松手状态。 */
    }

    if (beep_enable != 0U) /* 普通有效触摸需要蜂鸣；连续保活帧已在上方关闭该反馈。 */
    {
      SendKeyBeepMessage(1U); /* 只有事件成功进入业务队列后才给100ms单响，蜂鸣与主控真实接受结果一致。 */
    }
  }
}

#define SCREENKEY_MIN_FRAME_SIZE     9U  /* 最短按键包长度，单位字节；9 字节才能读到 frame[8] 的键值，不能随意减小。 */
#define SCREENKEY_FRAME_BUFFER_SIZE 16U  /* 单个按键包最多接收 16 字节；超长包丢弃，修改须同时检查屏幕协议和任务内存用量。 */

/* 主运行页各区域的按键顺序与 DWIN 0x2400~0x2404 表格一致，数组下标为 key-1。 */
static const uint8_t s_screen_handle_actions[] = {24U, 25U, 22U, 23U, 20U, 21U, 36U};
static const uint8_t s_screen_speed_actions[] = {30U, 31U, 32U, 33U};
static const uint8_t s_screen_direction_actions[] = {13U, 15U, 14U};
static const uint8_t s_screen_frequency_actions[] = {9U, 10U};
static const uint8_t s_screen_control_mode_actions[] = {16U, 17U, 18U, 43U};

/*
 * 函数功能：按 DWIN 区域内的 key 编号从固定映射表取得旧业务按键号。
 * 输入参数：actions 为映射表；action_count 为表项数；key_index 为从 1 开始的屏幕 key 编号。
 * 返回参数：返回映射后的旧业务按键号；越界时返回 0。
 */
static uint8_t ScreenKey_GetMappedAction(const uint8_t *actions,
                                         uint8_t action_count,
                                         uint8_t key_index)
{
  if ((key_index == 0U) || (key_index > action_count)) /* DWIN 编号从 1 开始，0 或超过表长都没有对应业务键。 */
  {
    return 0U; /* 屏幕按键编号越界时不投递任何业务消息。 */
  }

  return actions[key_index - 1U]; /* DWIN key 从 1 开始，数组从 0 开始。 */
}

/*
 * 函数功能：把屏幕 A/B 泵区按键映射到逻辑 A/B 泵业务按键。
 * 输入参数：section 为 0x05 或 0x06 泵区；key_index 为加、减、启停编号。
 * 返回参数：返回旧业务按键号；无效按键返回 0。
 */
static uint8_t ScreenKey_GetPumpAction(uint8_t section, uint8_t key_index)
{
  if ((key_index == 0U) || (key_index > 3U)) /* 每个泵区只定义加速、减速和启停三个触摸位置。 */
  {
    return 0U; /* 泵区只定义加、减、启停三个按键。 */
  }

#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
  if (section == 0x05U) /* 镜像安装下，屏幕左侧 0x05 区必须转到逻辑 B 泵。 */
  {
    static const uint8_t pump_b_actions[] = {5U, 6U, 11U};
    return pump_b_actions[key_index - 1U]; /* 镜像开启时屏幕原 A 区控制逻辑 B 泵。 */
  }
  else
  {
    static const uint8_t pump_a_actions[] = {7U, 8U, 12U};
    return pump_a_actions[key_index - 1U]; /* 镜像开启时屏幕原 B 区控制逻辑 A 泵。 */
  }
#else
  if (section == 0x05U) /* 默认安装下，屏幕左侧 0x05 区直接对应逻辑 A 泵。 */
  {
    static const uint8_t pump_a_actions[] = {7U, 8U, 12U};
    return pump_a_actions[key_index - 1U]; /* 默认屏幕 A 区控制逻辑 A 泵。 */
  }
  else
  {
    static const uint8_t pump_b_actions[] = {5U, 6U, 11U};
    return pump_b_actions[key_index - 1U]; /* 默认屏幕 B 区控制逻辑 B 泵。 */
  }
#endif
}

/*
 * 函数功能：投递脚踏定标页的低值、高值和中值存储事件。
 * 输入参数：key_index 为 DWIN 0x2420 区域内的按键编号。
 * 返回参数：无。
 */
static void ScreenKey_HandleCalibration(uint8_t key_index)
{
  switch (key_index)
  {
    case 0x01U:
      ScreenKey_LegacyEventPost(KEY_STORAGEMIN); /* 保存左侧低值。 */
      break;
    case 0x02U:
      ScreenKey_LegacyEventPost(KEY_STORAGEMAX); /* 保存左侧高值。 */
      break;
    case 0x03U:
      ScreenKey_LegacyEventPost(KEY_STORAGEMIN2); /* 保存右侧低值。 */
      break;
    case 0x04U:
      ScreenKey_LegacyEventPost(KEY_STORAGEMAX2); /* 保存右侧高值。 */
      break;
    case 0x07U:
      ScreenKey_LegacyEventPost(KEY_STORAMEDIAN); /* 保存左侧中值。 */
      break;
    case 0x08U:
      ScreenKey_LegacyEventPost(KEY_STORAMEDIAN2); /* 保存右侧中值。 */
      break;
    default:
      break; /* 未定义定标键不改变一次性旧事件。 */
  }
}

/*
 * 函数功能：按 DWIN 地址区和 key 编号分派一帧已经通过长度校验的屏幕按键。
 * 输入参数：frame 指向完整屏幕帧，至少包含 9 字节。
 * 返回参数：无。
 */
static void ScreenKey_DispatchFrame(const uint8_t *frame)
{
  uint8_t action = 0U; /* 0 表示当前地址没有对应业务动作。 */
  uint8_t section = frame[5]; /* DWIN 地址低字节决定主运行页功能区。 */
  uint8_t key_index = frame[8]; /* 第一个数据字的低字节保存该功能区内的按键编号。 */

  if (frame[4] == 0x55U) /* 地址高字节为 0x55 时，只检查触控按住信号，不按 0x24xx 按键区处理。 */
  {
    if (section == 0x20U) /* 仅 0x5520 是当前定义的触控保活地址，其它 0x55 地址不产生动作。 */
    {
      ScreenKey_PostLegacyAction(44U); /* 0x5520 为触控保活，按住期间持续运行。 */
    }
    return;
  }

  if ((frame[4] == 0x20U) && (section == 0x01U)) /* 0x2001 是 EX8 启动页脚踏定标入口。 */
  {
    if (key_index == 0x01U) /* 启动页只定义 key1 为脚踏定标按钮，其它值保持静默。 */
    {
      ScreenKey_LegacyEventPost(KEY_CONTINUOUSCLICK); /* 复用启动期一次性事件，不创建运行期业务队列。 */
    }
    return; /* 启动页事件只交给 UI_Start_Fun，不能进入主运行页按键映射。 */
  }

  if (frame[4] != 0x24U) /* 非主运行页地址在本分发器中没有业务按键。 */
  {
    return; /* 未定义地址的数据包直接忽略，不执行任何按键动作。 */
  }

  switch (section)
  {
    case 0x00U:
      action = ScreenKey_GetMappedAction(s_screen_handle_actions,
                                         (uint8_t)sizeof(s_screen_handle_actions),
                                         key_index);
      break;
    case 0x01U:
      action = ScreenKey_GetMappedAction(s_screen_speed_actions,
                                         (uint8_t)sizeof(s_screen_speed_actions),
                                         key_index);
      break;
    case 0x02U:
      action = ScreenKey_GetMappedAction(s_screen_direction_actions,
                                         (uint8_t)sizeof(s_screen_direction_actions),
                                         key_index);
      break;
    case 0x03U:
      action = ScreenKey_GetMappedAction(s_screen_frequency_actions,
                                         (uint8_t)sizeof(s_screen_frequency_actions),
                                         key_index);
      break;
    case 0x04U:
      action = ScreenKey_GetMappedAction(s_screen_control_mode_actions,
                                         (uint8_t)sizeof(s_screen_control_mode_actions),
                                         key_index);
      break;
    case 0x05U:
    case 0x06U:
      action = ScreenKey_GetPumpAction(section, key_index);
      break;
    case 0x07U:
      action = (key_index == 0x02U) ? 42U : 0U; /* 触控工作区 key2 退出触控。 */
      break;
    case 0x20U:
      ScreenKey_HandleCalibration(key_index);
      return;
    default:
      return; /* 未定义功能区不产生业务按键。 */
  }

  if (action != 0U) /* 地址和按键编号都有对应动作时，才继续检查此键是否允许使用。 */
  {
    ScreenKey_PostLegacyAction(action); /* 所有主运行页动作统一进入现有按键队列。 */
  }
}

/*
 * 函数功能：在本次 UART6 DMA 数据中逐字节寻找完整按键帧，并过滤短帧、超长帧和前导噪声。
 * 输入参数：data 为 DMA 数据副本；data_len 为本次实际收到的字节数。
 * 返回参数：无。
 */
static void ScreenKey_ParseRxData(uint8_t *data, uint16_t data_len)
{
  uint16_t offset = 0U;                                 /* 当前搜索位置，遇到噪声时只前进一个字节。 */
  uint16_t remaining = 0U;                              /* 当前搜索位置到有效数据末尾的真实剩余长度。 */
  uint16_t frame_len = 0U;                              /* 使用 16 位保存“长度字段+3”，避免 0xFD~0xFF 加法回绕。 */
  uint8_t frame[SCREENKEY_FRAME_BUFFER_SIZE] = { 0U }; /* 只存放已经通过边界检查的一帧数据。 */

  if (data == NULL) /* DMA 调用方未提供缓冲时不能继续访问接收数据。 */
  {
    return; /* 调用方没有提供数据缓冲时不访问内存，也不产生任何按键事件。 */
  }

  while ((data_len - offset) >= SCREENKEY_MIN_FRAME_SIZE) /* 不足 9 字节就停止；本函数没有保存或拼接跨次接收的半包。 */
  {
    remaining = data_len - offset; /* 每次按当前偏移重新计算，不能沿用未扣除噪声字节的总长度。 */

    if ((data[offset] != 0x5AU) || (data[offset + 1U] != 0xA5U) || /* 帧头或读变量命令不匹配时，把当前位置视为串口噪声。 */
        (data[offset + 3U] != 0x83U))
    {
      offset++; /* 当前字节不是合法帧头，继续寻找后面的 0x5A 0xA5，允许 DMA 数据带前导噪声。 */
      continue;
    }

    frame_len = (uint16_t)data[offset + 2U] + 3U; /* DWIN 长度字段不含两个帧头字节和自身，整帧需再加 3。 */
    if ((frame_len < SCREENKEY_MIN_FRAME_SIZE) || /* 声明长度过短、超过栈缓冲或尚未收全时都不能复制。 */
        (frame_len > SCREENKEY_FRAME_BUFFER_SIZE) ||
        (frame_len > remaining))
    {
      offset++; /* 长度声明异常时跳过当前伪帧头，继续寻找同一 DMA 数据中后续的合法按键帧。 */
      continue;
    }

    Common_CopyData(&data[offset], frame, frame_len);            /* 复制前已同时校验源区剩余长度和目标缓存容量。 */
    ScreenKey_DispatchFrame(frame);                              /* 按原有映射投递一个合法屏幕按键事件。 */
    Common_Memset(0U, frame, SCREENKEY_FRAME_BUFFER_SIZE);       /* 清掉上一帧内容，避免短数据字段沿用旧字节。 */
    offset += frame_len;                                         /* 完整帧按声明长度跳过，继续处理同一 DMA 包内的粘连帧。 */
  }
}

/*
 * 函数功能：读取 UART6 稳定 DMA 数据并解析本周期所有完整屏幕按键帧。
 * 输入参数：无。
 * 返回参数：无。
 */
void ScreenKey_Scan(void)
{
  uint16_t rlen = 0U;                              /* UART6 驱动返回的本次实际接收长度，最大 150 字节。 */
  uint8_t dat[UART6_MAX_PACKET_SIZE] = { 0U };     /* 保存 DMA 数据副本，驱动取数后会立即重启接收。 */

  rlen = Uart6_DMARecvDataPeek(dat);               /* 只读取已经连续三个检查周期不再增长的数据。 */
  if (rlen < SCREENKEY_MIN_FRAME_SIZE) /* 本周期数据不足最短按键帧时不进入解析器。 */
  {
    return; /* 少于 9 字节不可能包含当前业务按键帧，本周期不产生事件。 */
  }

  ScreenKey_ParseRxData(dat, rlen);                 /* 统一完成噪声跳过、长度检查和合法帧分发。 */
}

//============================================================================
//屏”按键“串口接收的任务初始化 22
//============================================================================
/* USER CODE BEGIN Header_SCREENKEYTaskFunc */
/**
* @brief Function implementing the SCREENKEYTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SCREENKEYTaskFunc */
/*
 * 函数功能：每 30ms 扫描一次屏幕串口按键，并维护触控运行保活超时。
 * 输入参数：event 为软件任务调度事件，本任务不区分事件值。
 * 返回参数：无。
 */
void SCREENKEYTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN SCREENKEYTaskFunc */
  /* Infinite loop */
	
  ScreenKey_Scan();
  ScreenKey_ServiceKeepAlive();
  /* USER CODE END SCREENKEYTaskFunc */
}

/*
 * 函数功能：创建屏幕按键任务并按 30ms 周期启动。
 * 输入参数：无。
 * 返回参数：无。
 */
void ScreenKey_ScanInit(void)
{
  /* definition and creation of SCREENKEYTask */
	Kernel_TaskCreate(&SCREENKEYTaskHandle, SCREENKEYTaskFunc);
	Kernel_TaskStart(&SCREENKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
