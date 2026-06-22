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

void ScreenKey_LegacyEventPost(uint8_t key_value)
{
  /* KEY_NONE 表示没有事件；非空键值只保留最后一次，行为与旧全局键值被覆盖的方式一致。 */
  if (key_value != KEY_NONE)
  {
    s_screenkey_legacy_event = key_value;
  }
}

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
static void ScreenKey_MarkTouchAlarmKeepAliveSeen(void)
{
  s_touch_alarm_release_ticks = 0U; /* 原始 0x5520 仍在持续发送，说明用户还没有松开触控按钮。 */
}

/*
 * 函数功能：判断本次 0x5520 保活帧是否应因报警锁存被拦截。
 * 输入参数：无。
 * 返回参数：true 表示本帧不投递业务队列；false 表示允许按普通触控保活处理。
 */
static uint8_t ScreenKey_ShouldBlockTouchKeepAliveByAlarm(void)
{
  if (WorkMessage.alarm_flag == true)
  {
    s_touch_alarm_release_required = 1U; /* 长按运行过程中出现真实报警后进入“必须松手”状态。 */
    ScreenKey_MarkTouchAlarmKeepAliveSeen(); /* 报警期间收到的本帧只能证明仍在按压，不能继续运行。 */
    return 1U; /* 报警帧不再投递到 ControlTypeActive，避免报警解除后同一次长按继续启动。 */
  }

  if (s_touch_alarm_release_required != 0U)
  {
    ScreenKey_MarkTouchAlarmKeepAliveSeen(); /* 报警已解除但原始保活仍在，继续等待用户松手。 */
    return 1U; /* 锁存未解除前不投递运行保活。 */
  }

  return 0U; /* 没有报警锁存时，0x5520 可按正常触控保活处理。 */
}

/*
 * 函数功能：周期检查 8 寸屏触控保活是否超时。
 * 输入参数：无。
 * 返回参数：无。
 */
static void ScreenKey_ServiceTouchKeepAlive(void)
{

  if (WorkMessage.hmiactive_work != 0U)
  {
    s_touch_keepalive_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 外控占用时不维护本机触控保活计时，避免外控手柄运行被 0x5520 超时逻辑停止。 */
    s_touch_alarm_release_required = 0U; /* 外控期间触控锁存直接视为空闲，退出外控后下一次触控重新开始计时。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 同步复位释放计数，避免外控结束后沿用旧触控长按状态。 */
    return; /* 外控虽然复用 TOUCHWORK 互斥标志，但不能进入本机触控保活状态机。 */
  }

  if ((WorkMessage.drivetype_work != TOUCHWORK) || (WorkMessage.touchactive_work != TOUCHWORK))
  {
    s_touch_keepalive_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 非触控模式不累计超时，避免脚踏/手控被误停。 */
    s_touch_alarm_release_required = 0U; /* 已经退出触控模式时清掉报警后松手锁存，下一次触控重新开始。 */
    s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 同步恢复释放计数到空闲态。 */
    return;
  }
if(WorkMessage.runflag_work == false)
{
  s_touch_keepalive_ticks=0;
  s_touch_alarm_release_ticks=0;

}


  if (s_touch_alarm_release_required != 0U)
  {
    if (s_touch_alarm_release_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS)
    {
      s_touch_alarm_release_ticks++; /* 锁存期间只有没有收到原始 0x5520 时才累计，持续按压会被接收函数清零。 */
    }
    if (s_touch_alarm_release_ticks >= SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS)
    {
      s_touch_alarm_release_required = 0U; /* 原始保活帧已经停止约 200ms，确认用户松手，可允许下一次按压。 */
    }
  }

  if (s_touch_keepalive_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS)
  {
    if(WorkMessage.runflag_work==true)
    s_touch_keepalive_ticks++; /* 30ms 任务每跑一次累计一次，连续未收到 0x5520 才判定松手。 */
    else
    s_touch_keepalive_ticks = 0U;
  }

  if (s_touch_keepalive_ticks >= SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS)
  {
     s_touch_alarm_release_required = 0U;
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
  uint8_t screen_key = 0U;

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
     // screen_key = SCREENKey_TouchEXIT;
      break;

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

  if (screen_key != 0U)
  {
    uint8_t beep_enable = 1U; /* 默认所有有效触控按键响一声，给操作者明确反馈。 */
    if (screen_key == SCREENKey_TouchKeepAlive)
    {
      if (ScreenKey_ShouldBlockTouchKeepAliveByAlarm() != 0U)
      {
        return; /* 报警锁存期间屏幕仍在长按时不蜂鸣、不投递，必须松手后下一次按压才有效。 */
      }
      if (s_touch_keepalive_ticks < SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS)
      {
        beep_enable = 0U; /* 0x5520 连续保活帧不重复蜂鸣，只在刚按下或超时后重新按下时响一次。 */
      }
      ScreenKey_ResetTouchKeepAlive(); /* 保活帧进入业务队列前先清本地超时计数，防止队列调度延迟造成误停。 */
    }
    else if (screen_key == SCREENKey_TouchEXIT)
    {
      s_touch_alarm_release_required = 0U; /* 用户主动退出触控时视为已松手，清除报警锁存。 */
      s_touch_alarm_release_ticks = SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_TICKS; /* 下一次进入触控重新计算报警后松手状态。 */
    }
    if (beep_enable != 0U)
    {
      SendKeyBeepMessage(1U); /* 屏幕有效触控已被主控解析，先给 100ms 单响反馈，再交给业务队列执行。 */
    }
    SendKeyBehMessage(SCREENKey, screen_key);
  }
}

//============================================================================
//1.屏”按键“
//============================================================================

//============================================================================
// 函数名称: ScreenKey_Scan()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 长按操作150ms
//============================================================================
void ScreenKey_Scan(void)
{
  uint8_t rlen = 0, slen = 0, i = 0, len = 0;
  uint8_t dat[UART6_MAX_PACKET_SIZE] = { 0 }, dat1[16] = { 0 };

  //读取串口数据
  rlen = Uart6_DMARecvDataPeek(dat);
  if (rlen < 9)   //不够一个数据包大小
    return;

  slen = rlen;

  //查询本帧数据包的帧头0x5A 0xA5
  for (i = 0; i < (rlen - 8); i++)
  {
	  if ((dat[i] == 0x5A) && (dat[i + 1] == 0xA5) && (dat[i + 3] == 0x83))  //帧头 指令
	  {
	    len = dat[i + 2] + 3;

	    if (slen < len)  //剩余长度应满足数据帧长度
		    break ;

	    Common_CopyData(&dat[i], dat1, len);    //截取数据

	    //键值...
	    switch (dat1[4])
	    {
		    case 0x20 :  //第一幅图“LOGO连续点击”进入管理者模式 0_开机界面
		    {
		      /* 新屏不再保留老屏入口，启动页 0x2001 只消费串口帧不进入业务。 */
		    }
		   break;
		  	case 0x24 :  // 
		    {
		      switch (dat1[5])
		      {				
            case 0x00 : // 主运行页顶部：手柄、开口定位、磨/刨、自动识别
						{
			         switch (dat1[8])
							 {
									case 0x01 : ScreenKey_PostLegacyAction(24U);  break;//A 手柄
									case 0x02 : ScreenKey_PostLegacyAction(25U);  break;//B 手柄
									case 0x03 : ScreenKey_PostLegacyAction(22U); break;	//开口定位减
									case 0x04 : ScreenKey_PostLegacyAction(23U); break;	//开口定位加
									case 0x05 : ScreenKey_PostLegacyAction(20U); break;	//选择磨头模式
									case 0x06 : ScreenKey_PostLegacyAction(21U); break;	//选择刨刀模式
									case 0x07 : ScreenKey_PostLegacyAction(36U); break;	//自动识别刀具
									default : break;
							 }
             }break; 
            case 0x01 : // 主运行页速度：快减、慢减、慢加、快加
						{
			         switch (dat1[8])
							 {
									case 0x01 : ScreenKey_PostLegacyAction(30U); break;	//EX8 表格 key1 是快减，按当前方向步进两倍减少
									case 0x02 : ScreenKey_PostLegacyAction(31U); break;  //EX8 表格 key2 是慢减，按当前方向步进减少
									case 0x03 : ScreenKey_PostLegacyAction(32U); break;	//EX8 表格 key3 是慢加，按当前方向步进增加
									case 0x04 : ScreenKey_PostLegacyAction(33U); break;	//EX8 表格 key4 是快加，按当前方向步进两倍增加
									default : break;
							 }
						}break;						
            case 0x02 :// 主运行页方向：正转、往复、反转
						{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(13U); break;    //正转
										case 0x02 : ScreenKey_PostLegacyAction(15U); break;    //往复
										case 0x03 : ScreenKey_PostLegacyAction(14U); break;    //反转
										default : break;
									}
							}break;
            case 0x03 :// 主运行页频率：减、加
								{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(9U); break;      //EX8 表格 key1 是频率加
										case 0x02 : ScreenKey_PostLegacyAction(10U);	 break;      //EX8 表格 key2 是频率减
										default : break;
									}
								}break;
						case 0x04 :// 主运行页控制方式：脚控、手控、触控、外部通信
									{ 
										switch (dat1[8])
										{
											case 0x01 : ScreenKey_PostLegacyAction(16U); break;    //脚控
											case 0x02 : ScreenKey_PostLegacyAction(17U); break;    //手控
											case 0x03 : ScreenKey_PostLegacyAction(18U); break;    //触控
											case 0x04 : ScreenKey_PostLegacyAction(43U); break;    //外部通信/外控退出
											default : break;
										}
								}break;	
            case 0x05 :// A 泵加、减、启停
						{

								switch (dat1[8])
								{
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
									case 0x01 : ScreenKey_PostLegacyAction(5U); break;  // 显示镜像开启时，屏幕原 A 区实际对应逻辑 B 泵加。
									case 0x02 : ScreenKey_PostLegacyAction(6U); break;  // 显示镜像开启时，屏幕原 A 区实际对应逻辑 B 泵减。
									case 0x03 : ScreenKey_PostLegacyAction(11U); break; // 显示镜像开启时，屏幕原 A 区实际对应逻辑 B 泵启停。
#else
									case 0x01 : ScreenKey_PostLegacyAction(7U); break;  //A 泵加
									case 0x02 : ScreenKey_PostLegacyAction(8U); break;  //A 泵减
									case 0x03 : ScreenKey_PostLegacyAction(12U); break; //A 泵启停
#endif
									default : break;
								}
						}break;

					case 0x06 :// B 泵加、减、启停
								{ 
									switch (dat1[8])
									{
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
										case 0x01 : ScreenKey_PostLegacyAction(7U); break;  // 显示镜像开启时，屏幕原 B 区实际对应逻辑 A 泵加。
										case 0x02 : ScreenKey_PostLegacyAction(8U); break;  // 显示镜像开启时，屏幕原 B 区实际对应逻辑 A 泵减。
										case 0x03 : ScreenKey_PostLegacyAction(12U); break; // 显示镜像开启时，屏幕原 B 区实际对应逻辑 A 泵启停。
#else
										case 0x01 : ScreenKey_PostLegacyAction(5U); break;  //B 泵加
										case 0x02 : ScreenKey_PostLegacyAction(6U); break;  //B 泵减
										case 0x03 : ScreenKey_PostLegacyAction(11U); break; //B 泵启停
#endif
										default : break;
									}
							}break;
            case 0x07 ://  触控工作区：key2 为触控退出
						{
							switch (dat1[8])
							{
								case 0x02 : ScreenKey_PostLegacyAction(42U); break;  //触控退出，释放屏幕控制
								default : break;
							}
						}break;
            case 0x20 ://  定标按键
						{ 
							switch (dat1[8])
							{
								case 0x01 : ScreenKey_LegacyEventPost(KEY_STORAGEMIN); break;  //存储小值(低值左边)
								case 0x02 : ScreenKey_LegacyEventPost(KEY_STORAGEMAX); break;  //存储大值（高值左边）
								case 0x03 : ScreenKey_LegacyEventPost(KEY_STORAGEMIN2); break;  //存储小值2（低值右边）
								case 0x04 : ScreenKey_LegacyEventPost(KEY_STORAGEMAX2); break;  //存储大值2（高值右边）
								case 0x07 : ScreenKey_LegacyEventPost(KEY_STORAMEDIAN); break;  //存储中间值（左边中间）
								case 0x08 : ScreenKey_LegacyEventPost(KEY_STORAMEDIAN2); break;  //存储中间值2（右边中间）
								
								default : break;
							}
						}break; 

						default : break;
					}
		    }
		    break;
			case 0x55:
				 switch (dat1[5])
					{
						case 0x20 : ScreenKey_PostLegacyAction(44U); break;  //触控保活，按住期间持续运行
            //ScreenKey_ResetTouchKeepAlive();

					//	case 0x30 : ScreenKey_PostLegacyAction(42U); break;  //触控停止
					}
					break;
		    default : break;
				
	    }

//	    //参数设置---键值范围
//	      ScreenKey_ParamSet();

	    Common_Memset(0, dat1, 15);
  	  i += (len - 1);
	    slen -= len;
	  }
  }
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
void SCREENKEYTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN SCREENKEYTaskFunc */
  /* Infinite loop */
	
  ScreenKey_Scan();
  ScreenKey_ServiceTouchKeepAlive();
  /* USER CODE END SCREENKEYTaskFunc */
}

void ScreenKey_ScanInit(void)
{
  /* definition and creation of SCREENKEYTask */
	Kernel_TaskCreate(&SCREENKEYTaskHandle, SCREENKEYTaskFunc);
	Kernel_TaskStart(&SCREENKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
