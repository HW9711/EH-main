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
/* 触控保活超时按 7 个 30ms 扫描周期处理，屏幕停止发送 0x5520 后约 200ms 停止电机输出。 */
#define SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS 14U
/* 触控保活计数器只在触控模式下递增，收到 0x5520 后清零，避免触控按钮松开后电机继续运行。 */
static uint8_t s_touch_keepalive_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS;
/* 触控长按过程中发生报警后置位，必须等屏幕停止发送 0x5520 一段时间才允许再次运行。 */
static uint8_t s_touch_alarm_release_required = 0U;
/* 报警锁存期间的原始保活帧间隔计数，持续收到 0x5520 时清零，只有真正松手才增长到超时。 */
static uint8_t s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS;

/*
 * 函数功能：保存启动页或脚踏定标页的一次性旧按键事件。
 * 输入参数：key_value 为旧页面按键编号；KEY_NONE 表示没有事件。
 * 返回参数：无。
 */
void ScreenKey_LegacyEventPost(uint8_t key_value)
{
  /* KEY_NONE 表示没有事件；非空键值只保留最后一次，行为与旧全局键值被覆盖的方式一致。 */
  if (key_value != KEY_NONE) /* 空事件不能覆盖尚未被页面任务消费的有效按键。 */
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

  /* 读取后立即清空，保证启动页和定标页不会重复消费同一次屏幕/脚踏事件。 */
  s_screenkey_legacy_event = KEY_NONE;

  return key_value;
}

/*
 * 函数功能：复位 8 寸屏触控保活计数。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_ResetTouchKeepAlive(void)
{
  s_touch_keepalive_ticks = 0U; /* 收到 0x5520 保活帧时从 0 重新计数，保证按压期间电机持续运行。 */
}

/*
 * 函数功能：记录报警锁存期间仍然收到触控保活帧。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_MarkAlarmKeepAlive(void)
{
  s_touch_alarm_release_ticks = 0U; /* 原始 0x5520 仍在持续发送，说明用户还没有松开触控按钮。 */
}

/*
 * 函数功能：判断本次 0x5520 保活帧是否应因报警锁存被拦截。
 * 输入参数：无。
 * 返回参数：true 表示本帧不投递业务队列；false 表示允许按普通触控保活处理。
 */
static uint8_t ScreenKey_BlockAlarmKeepAlive(void)
{
  if (WorkMessage.alarm_flag == true) /* 当前存在真实报警时，持续触摸不能继续维持电机运行。 */
  {
    s_touch_alarm_release_required = 1U; /* 长按运行过程中出现真实报警后进入“必须松手”状态。 */
    ScreenKey_MarkAlarmKeepAlive(); /* 报警期间收到的本帧只能证明仍在按压，不能继续运行。 */
    return 1U; /* 报警帧不再投递到 ControlTypeActive，避免报警解除后同一次长按继续启动。 */
  }

  if (s_touch_alarm_release_required != 0U) /* 报警虽已解除，但用户尚未松手时仍要保持运行锁存。 */
  {
    ScreenKey_MarkAlarmKeepAlive(); /* 报警已解除但原始保活仍在，继续等待用户松手。 */
    return 1U; /* 锁存未解除前不投递运行保活。 */
  }

  return 0U; /* 没有报警锁存时，0x5520 可按正常触控保活处理。 */
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
    s_touch_keepalive_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 外控占用时不维护本机触控保活计时，避免外控手柄运行被 0x5520 超时逻辑停止。 */
    s_touch_alarm_release_required = 0U; /* 外控期间触控锁存直接视为空闲，退出外控后下一次触控重新开始计时。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 同步复位释放计数，避免外控结束后沿用旧触控长按状态。 */
    return; /* 外控虽然复用 TOUCHWORK 互斥标志，但不能进入本机触控保活状态机。 */
  }

  if ((WorkMessage.drivetype_work != TOUCHWORK) || (WorkMessage.touchactive_work != TOUCHWORK)) /* 只有本机触控已激活才进入保活状态机。 */
  {
    s_touch_keepalive_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 非触控模式不累计超时，避免脚踏/手控被误停。 */
    s_touch_alarm_release_required = 0U; /* 已经退出触控模式时清掉报警后松手锁存，下一次触控重新开始。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 同步恢复释放计数到空闲态。 */
    return; /* 非触控控制源不能被本任务的保活超时逻辑停止。 */
  }

  if ((WorkMessage.runflag_work == false) && (s_touch_alarm_release_required == 0U)) /* 触控界面已打开但尚未按住运行时保持空闲计数。 */
  {
    s_touch_keepalive_ticks = 0U; /* 普通触控待运行状态下保持保活计数为 0，避免未按运行键时触发超时停机。 */
    s_touch_alarm_release_ticks = 0U; /* 没有报警释放等待时才清松手计数，避免运行中掉线后永远等不到松手确认。 */
  }


  if (s_touch_alarm_release_required != 0U) /* 报警后必须检测到一段无保活时间，才认定用户已经松手。 */
  {
    if (s_touch_alarm_release_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS) /* 未到松手确认时间时继续累计空闲周期。 */
    {
      s_touch_alarm_release_ticks++; /* 锁存期间只有没有收到原始 0x5520 时才累计，持续按压会被接收函数清零。 */
    }
    if (s_touch_alarm_release_ticks >= SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS) /* 连续约 200ms 未收到保活，确认本次按压已经释放。 */
    {
      s_touch_alarm_release_required = 0U; /* 原始保活帧已经停止约 200ms，确认用户松手，可允许下一次按压。 */
      Pubinterface_ReleaseTouchHandleNotConnectedAlarm(); /* 若本次松手对应运行中拔手柄报警，则退出触控控制源并关闭报警弹窗。 */
    }
  }

  if (s_touch_keepalive_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS) /* 尚未超时时只在电机实际运行期间累计。 */
  {
    if (WorkMessage.runflag_work == true) /* 运行态需要持续收到 0x5520，缺帧才允许触发自动停机。 */
    {
      s_touch_keepalive_ticks++; /* 30ms 任务每跑一次累计一次，连续未收到 0x5520 才判定松手。 */
    }
    else /* 待运行态没有电机输出，不需要累计停机超时。 */
    {
      s_touch_keepalive_ticks = 0U; /* 清零后下一次按压从完整保活窗口开始。 */
    }
  }

  if (s_touch_keepalive_ticks >= SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS) /* 运行态连续缺少保活达到阈值时执行安全停机。 */
  {
    s_touch_alarm_release_required = 0U; /* 普通松手停机不需要继续保持报警后松手锁存。 */
    Pubinterface_StopTouchKeepAliveRun(); /* 超时只停电机输出，不退出触控模式，屏幕仍保持触控入口状态。 */
  }
}

/*
 * 屏幕串口协议仍沿用旧的页面地址和按键编号，但业务出口改为 V1.8 新接口事件。
 * 这里集中维护旧 `ScreenKey_data` 数字到 `SCREENKey_*` 枚举的映射：
 * 1. 解析层继续按原 HMI 帧格式识别按键，避免改动串口协议；
 * 2. 行为层统一交给 sscKEYBH 分发，逐步替代旧屏幕模块的按键仓库职责；
 * 3. 无新接口等价项的旧码暂时静默，后续迁 UI/RFID 时再补专用事件。
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

    case 44U: /* ScreenKey_TouchKeepAlive：脚本验收标记，实际业务宏名保持 SCREENKey_TouchKeepAlive。 */
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
    if (screen_key == SCREENKey_TouchKeepAlive) /* 触控运行保活需要先处理报警后的“必须松手”锁存。 */
    {
      if (ScreenKey_BlockAlarmKeepAlive() != 0U) /* 报警期间或报警后仍在长按时，当前保活帧必须静默丢弃。 */
      {
        return; /* 报警锁存期间屏幕仍在长按时不蜂鸣、不投递，必须松手后下一次按压才有效。 */
      }
    }

    if (ScreenKey_CanUse(screen_key) == false) /* 当前图标必须处于可操作状态，黑色或隐藏图片不接受触摸。 */
    {
      return; /* 黑色、隐藏或当前业务状态不可用的坐标静默丢弃，既不蜂鸣也不进入业务队列。 */
    }

    if (screen_key == SCREENKey_TouchKeepAlive) /* 只有触控运行保活需要维护连续帧超时和重复蜂鸣。 */
    {
      if (s_touch_keepalive_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS) /* 计数尚未超时说明仍是同一次长按，不重复鸣叫。 */
      {
        beep_enable = 0U; /* 0x5520 连续保活帧不重复蜂鸣，只在刚按下或超时后重新按下时响一次。 */
      }
      ScreenKey_ResetTouchKeepAlive(); /* 可用保活帧进入业务队列前清本地超时计数，隐藏触控区的误报不会延长运行。 */
    }
    else if (screen_key == SCREENKey_TouchEXIT) /* 用户主动退出触控时，立即结束报警后的松手等待。 */
    {
      s_touch_alarm_release_required = 0U; /* 用户主动退出触控时视为已松手，清除报警锁存。 */
      s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 下一次进入触控重新计算报警后松手状态。 */
    }
    if (beep_enable != 0U) /* 普通有效触摸需要蜂鸣；连续保活帧已在上方关闭该反馈。 */
    {
      SendKeyBeepMessage(1U); /* 屏幕有效触控已被主控解析，先给 100ms 单响反馈，再交给业务队列执行。 */
    }
    SendKeyBehMessage(SCREENKey, screen_key);
  }
}

#define SCREENKEY_MIN_FRAME_SIZE     9U  /* 当前按键帧至少包含帧头、命令、地址和 dat1[8] 键值。 */
#define SCREENKEY_FRAME_BUFFER_SIZE 16U  /* 保持原局部帧缓存容量，超长声明帧直接丢弃，禁止覆盖任务栈。 */

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
  uint8_t key_index = frame[8]; /* 数据区第一个字节是该功能区内的按键编号。 */

  if (frame[4] == 0x55U) /* 0x55 地址族只承载触控运行保活，不按主页面 0x24 区域解析。 */
  {
    if (section == 0x20U) /* 仅 0x5520 是当前定义的触控保活地址，其它 0x55 地址不产生动作。 */
    {
      ScreenKey_PostLegacyAction(44U); /* 0x5520 为触控保活，按住期间持续运行。 */
    }
    return;
  }

  if (frame[4] != 0x24U) /* 非主运行页地址在本分发器中没有业务按键。 */
  {
    return; /* 启动页 0x20 和其它未定义地址只消费串口帧，不进入业务。 */
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

  if (action != 0U) /* 只有地址和 key 均匹配映射表时才进入统一按键门禁。 */
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

  while ((data_len - offset) >= SCREENKEY_MIN_FRAME_SIZE) /* 剩余数据不足最短帧时停止扫描，半帧等待下一次 DMA 接收。 */
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
