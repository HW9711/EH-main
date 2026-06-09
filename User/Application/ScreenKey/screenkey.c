//screenkey.c

#include "screenkey.h"
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
      screen_key = SCREENKey_FREQ_Add;
      break;

    case 10U:
      screen_key = SCREENKey_FREQ_Sub;
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
      screen_key = SCREENKey_TouchEXIT;
      break;

    case 41U:
      screen_key = SCREENKey_TouchStart;
      break;

    case 42U:
      screen_key = SCREENKey_TouchEXIT;
      break;

    case 43U:
      screen_key = SCREENKey_HMI_EXIT;
      break;

    case 30U:
      screen_key = SCREENKey_SPEED_Sub_Large; /* 新屏速度左侧大减键，固定减少 10000。 */
      break;

    case 31U:
      screen_key = SCREENKey_SPEED_Sub_Small; /* 新屏速度左侧小减键，固定减少 1000。 */
      break;

    case 32U:
      screen_key = SCREENKey_SPEED_Add_Small; /* 新屏速度右侧小加键，固定增加 1000。 */
      break;

    case 33U:
      screen_key = SCREENKey_SPEED_Add_Large; /* 新屏速度右侧大加键，固定增加 10000。 */
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
    SendKeyBeepMessage(1U); /* 屏幕有效触控已被主控解析，先给 100ms 单响反馈，再交给业务队列执行。 */
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
            case 0x01 : // 主运行页速度：大减、小减、小加、大加
						{
			         switch (dat1[8])
							 {
									case 0x01 : ScreenKey_PostLegacyAction(30U); break;	//速度大幅减少 10000
									case 0x02 : ScreenKey_PostLegacyAction(31U); break;  //速度小幅减少 1000
									case 0x03 : ScreenKey_PostLegacyAction(32U); break;	//速度小幅增加 1000
									case 0x04 : ScreenKey_PostLegacyAction(33U); break;	//速度大幅增加 10000
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
										case 0x01 : ScreenKey_PostLegacyAction(10U); break;      //频率减
										case 0x02 : ScreenKey_PostLegacyAction(9U);	 break;      //频率加
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
									case 0x01 : ScreenKey_PostLegacyAction(7U); break;  //A 泵加
									case 0x02 : ScreenKey_PostLegacyAction(8U); break;  //A 泵减
									case 0x03 : ScreenKey_PostLegacyAction(12U); break; //A 泵启停
									default : break;
								}
						}break;

					case 0x06 :// B 泵加、减、启停
								{ 
									switch (dat1[8])
									{
										case 0x01 : ScreenKey_PostLegacyAction(5U); break;  //B 泵加
										case 0x02 : ScreenKey_PostLegacyAction(6U); break;  //B 泵减
										case 0x03 : ScreenKey_PostLegacyAction(11U); break; //B 泵启停
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
		    case 0x51 :  //A泵  
		    {
		      switch (dat1[5])
		      {  
						///////////////////A泵/////////////////////////

						default : break;
	        }
		    }
		    break;
		    case 0x52 :   //B泵  
		    {
		      switch (dat1[5])
		      { ///////////////////B泵/////////////////////////

						default : break;
	        }
		    }
		    break;
		    case 0x53 :  //转速  
		    {
		      switch (dat1[5])
		      {
		        case 0x10 : ScreenKey_PostLegacyAction(2U); break;  //速度减    按压一次

			      case 0x50 : ScreenKey_PostLegacyAction(4U); break;  //速度加    按压一次
			      default : break;
		      }
		    }
		    break;	

			case 0x54 :  //转速  
		    {
		      switch (dat1[5])
		      {
		        case 0x10 : ScreenKey_PostLegacyAction(1U); break;  //速度减    按压一次

			      case 0x50 : ScreenKey_PostLegacyAction(3U); break;  //速度加    按压一次
			      default : break;
		      }
		    }
		    break;
			case 0x55:
				 switch (dat1[5])
					{
						 case 0x10 : ScreenKey_PostLegacyAction(41U); break;  //触控启动
						case 0x30 : ScreenKey_PostLegacyAction(42U); break;  //触控停止
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
  /* USER CODE END SCREENKEYTaskFunc */
}

void ScreenKey_ScanInit(void)
{
  /* definition and creation of SCREENKEYTask */
	Kernel_TaskCreate(&SCREENKEYTaskHandle, SCREENKEYTaskFunc);
	Kernel_TaskStart(&SCREENKEYTaskHandle, KERNEL_TASK_ALWAYS, 30);
}
