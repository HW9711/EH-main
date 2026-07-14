//motoruartdata.c

#include "motoruartdata.h"
#include "common.h"
#include "uart1.h"
#include "pump.h"
#include <string.h>
#include "kernel_scheduler.h"
#include "Pubinterface.h"
#include "sscBEEP.h"
#include "sscUIDP.h"

kernel_task_t MOTORUARTTaskHandle;

#define MOTOR_UART_DRIVER_ERR_NONE       0U  /* 参考驱动 Err=0：无故障，dat1[7] 恢复到该值时允许释放本模块设置的报警。 */
#define MOTOR_UART_DRIVER_ERR_FAIL       1U  /* 参考驱动 Err=1：模块保护，主控按驱动板故障处理。 */
#define MOTOR_UART_DRIVER_ERR_OC1        2U  /* 参考驱动 Err=2：过流保护，主控按电机过载/刀具卡住处理。 */
#define MOTOR_UART_DRIVER_ERR_OV         3U  /* 参考驱动 Err=3：过压保护，主控按系统供电电压异常处理。 */
#define MOTOR_UART_DRIVER_ERR_UV         4U  /* 参考驱动 Err=4：欠压保护，主控按系统供电电压异常处理。 */
#define MOTOR_UART_DRIVER_ERR_RUNSTALL   5U  /* 参考驱动 Err=5：运行中堵转，主控按电机过载/刀具卡住处理。 */
#define MOTOR_UART_DRIVER_ERR_TEMP       6U  /* 参考驱动 Err=6：驱动器过温，当前报警表无独立温度项，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_SAVE       7U  /* 参考驱动 Err=7：参数保存错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_OVR        8U  /* 参考驱动 Err=8：刹车时间过长，不是电压故障，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_ENOC       9U  /* 参考驱动 Err=9：编码器错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_START      10U /* 参考驱动 Err=10：开环检测错误，归入驱动板故障。 */
#define MOTOR_UART_DRIVER_ERR_NOHALL     11U /* 参考驱动 Err=11：霍尔断线，主控按霍尔错误处理。 */
#define MOTOR_UART_DRIVER_ERR_SDAHALL    12U /* 参考驱动 Err=12：霍尔学习错误，主控按霍尔错误处理。 */
#define MOTOR_UART_DRIVER_ERR_HANDSHAKE  13U /* 参考驱动 Err=13：通信握手错误，主控按驱动板故障处理。 */
#define MOTOR_UART_DRIVER_ERR_PHASELOSS  14U /* 参考驱动 Err=14：缺相，主控按电机相位错误处理。 */
#define MOTOR_UART_DRIVER_ERR_POSDRAG    15U /* 参考驱动 Err=15：有 Hall 拖动错误，归入驱动板故障。 */

#define MOTOR_UART_ALARM_PHASE           3U  /* UI 绑定报警码 0x03：电机相位错误。 */
#define MOTOR_UART_ALARM_HALL            4U  /* UI 绑定报警码 0x04：电机霍尔错误。 */
#define MOTOR_UART_ALARM_OVERLOAD        5U  /* UI 绑定报警码 0x05：电机过载或刀具卡住。 */
#define MOTOR_UART_ALARM_VOLTAGE         8U  /* UI 绑定报警码 0x08：系统供电电压不稳定。 */
#define MOTOR_UART_ALARM_DRIVER_BOARD    11U /* UI 绑定报警码 0x0B：驱动板故障。 */

static uint8_t s_motor_uart_alarm_owned = 0U;        /* 记录本模块最近一次写入的报警码，驱动恢复正常时只清自己拥有的报警。 */
static uint8_t s_motor_uart_last_driver_error = 0U;  /* 记录上一帧驱动 Err，避免同一个故障每帧重复触发蜂鸣和上位机弹窗。 */
static uint32_t s_motor_uart_alarm_start_tick = 0U;  /* 记录驱动报警第一次弹出的系统 tick，用于计算 3 秒保持时间。 */

/*
 * UART2 已由 ExternalComm 独立任务接管。
 * 本模块只保留 UART1 驱动板接收解析，避免旧 6 字节 HMI 短帧再次读取 UART2 DMA 缓冲。
 */
static void MotorUart_SetAlarm(uint8_t alarm_value)
{
	uint8_t display_value[10] = {0U};     /* UI_AIARM_ID 只读取 Value[0]，其余补零避免残留旧报警参数。 */

	display_value[0] = alarm_value;       /* 把驱动细分报警码直接传给 UI 绑定表。 */
	WorkAlarm_Set(alarm_value);           /* 统一报警状态仍由 WorkAlarm_Set 维护，避免直接改 WorkMessage。 */
	SendAlarmMessage(alarm_value);         /* 同步蜂鸣任务进入对应报警声。 */
	SendUIDSMessage(UI_AIARM_ID, true, display_value); /* 同步屏幕报警弹窗，报警码仍来自统一 WorkMessage 体系。 */
}

static uint8_t MotorUart_MapDriverErrorToAlarm(uint8_t driver_error)
{
	switch (driver_error)
	{
		case MOTOR_UART_DRIVER_ERR_OC1:
		case MOTOR_UART_DRIVER_ERR_RUNSTALL:
			return MOTOR_UART_ALARM_OVERLOAD; /* 过流和堵转都属于“电机过载/刀具卡住”这类现场可处理故障。 */

		case MOTOR_UART_DRIVER_ERR_OV:
		case MOTOR_UART_DRIVER_ERR_UV:
			return MOTOR_UART_ALARM_VOLTAGE; /* 过压/欠压按 UI 绑定显示系统供电电压异常。 */

		case MOTOR_UART_DRIVER_ERR_NOHALL:
		case MOTOR_UART_DRIVER_ERR_SDAHALL:
			return MOTOR_UART_ALARM_HALL; /* 霍尔断线/学习错误均走 0x04 霍尔错误报警。 */

		case MOTOR_UART_DRIVER_ERR_PHASELOSS:
			return MOTOR_UART_ALARM_PHASE; /* 缺相比普通驱动故障更具体，优先映射到相位错误。 */

		case MOTOR_UART_DRIVER_ERR_NONE:
			return WORK_ALARM_NONE; /* 无故障不产生报警，调用方会负责释放本模块拥有的旧报警。 */

		default:
			return MOTOR_UART_ALARM_DRIVER_BOARD; /* 保存错误、刹车超时、握手错误等统一提示驱动板故障。 */
	}
}

/*
 * 函数功能：把驱动板 Err 编码转换为主控报警，并记录本次弹窗开始时间。
 * 输入参数：driver_error 为驱动板回包 dat1[7] 的错误码。
 * 返回参数：无。
 */
static void MotorUart_SetDriverAlarm(uint8_t driver_error)
{
	/* 运行手柄掉线属于当前首要故障，驱动报警不能覆盖该报警，否则用户将失去掉线确认入口。 */
	if (WorkMessage.alarm_value == WORK_ALARM_HANDLE_NOT_CONNECTED) /* 运行手柄掉线报警必须保留用户确认入口，驱动故障不能覆盖。 */
	{
		return;
	}
	uint8_t alarm_value = MotorUart_MapDriverErrorToAlarm(driver_error); /* 把参考驱动 Err 编码转换为主控统一报警码。 */

	if (alarm_value == 0U)
	{
		return; /* 防御：无故障不应调用设置报警，直接返回避免误清其他模块状态。 */
	}

	if ((s_motor_uart_last_driver_error == driver_error) && /* 同一故障已由本模块持有且屏幕仍显示时，不重复上报。 */
	    (s_motor_uart_alarm_owned == alarm_value) &&
	    (WorkMessage.alarm_flag == true) &&
	    (WorkMessage.alarm_value == alarm_value))
	{
		return; /* 同一个驱动故障已上报过，本帧不重复投递蜂鸣/上位机报警。 */
	}

	s_motor_uart_last_driver_error = driver_error; /* 记录真实驱动 Err，便于下一帧判断是否发生了故障变化。 */
	s_motor_uart_alarm_owned = alarm_value;        /* 记录本模块拥有的主控报警码，后续驱动恢复正常时才允许自动清除。 */
	s_motor_uart_alarm_start_tick = HAL_GetTick(); /* 新报警出现时记录弹窗起点，后续即使 Err 很快恢复也要显示满 3 秒。 */
	MotorUart_SetAlarm(alarm_value);               /* 同步 WorkMessage、蜂鸣任务和外部通信报警上传。 */
}

/*
 * 函数功能：驱动 Err 恢复为 0 后，延时到 3 秒保持时间结束再清除本模块拥有的报警弹窗。
 * 输入参数：无。
 * 返回参数：无。
 */
static void MotorUart_ClearDriverAlarmIfOwned(void)
{
	if ((s_motor_uart_alarm_owned != 0U) && /* 只清除本模块持有且当前仍显示的驱动报警，不能误清其它报警。 */
	    (WorkMessage.alarm_flag == true) &&
	    (WorkMessage.alarm_value == s_motor_uart_alarm_owned))
	{
		if ((uint32_t)(HAL_GetTick() - s_motor_uart_alarm_start_tick) < ALARM_DRV_MS)
		{
			return; /* 驱动 Err 已经恢复但 3 秒显示时间未到，继续保持 WorkMessage 和屏幕报警，避免现场只看到闪屏。 */
		}

		WorkAlarm_Clear();                 /* 驱动 Err 已恢复且弹窗已满 3 秒，释放本模块写入的报警码。 */
		SendAlarmMessage(WORK_ALARM_NONE); /* 3 秒提示结束后同步释放蜂鸣，避免蜂鸣锁存继续保持。 */
		SendUIDSMessage(UI_AIARM_ID, false, NULL); /* 3 秒提示结束后关闭屏幕报警弹窗，NULL 参数由 UIDP 统一补零。 */
	}

	s_motor_uart_alarm_owned = 0U;        /* 无论当前报警是否被其他模块接管，都释放本模块报警所有权。 */
	s_motor_uart_last_driver_error = 0U;  /* 驱动恢复正常后清掉上一次真实 Err，下一次新故障可以重新上报。 */
	s_motor_uart_alarm_start_tick = 0U;   /* 本次保持周期结束或报警归属已转移，清掉旧 tick 防止下次沿用。 */
}

/*
 * 函数功能：驱动故障恢复边沿撤销所有电机控制源的运行请求，防止旧请求自动恢复。
 * 输入参数：无，函数直接清理 WorkMessage 和 ControlSignalMessage。
 * 返回参数：无。
 */
static void MotorUart_StopAllWork(void)
{
	WorkMessage.runflag_work = false; /* 撤销电机运行命令，驱动任务下一周期继续保持停机。 */

	ControlSignalMessage.handle_control_flag = false; /* 清除手柄按键运行来源，避免故障恢复后再次置位。 */
	ControlSignalMessage.HMI_control_flag = false; /* 清除外控运行来源，要求上位机重新下发启动。 */
	ControlSignalMessage.jtL_control_flag = false; /* 清除左脚踏运行来源，要求用户松开后重新踩下。 */
	ControlSignalMessage.jtR_control_flag = false; /* 清除右脚踏运行来源，保持双脚踏停机状态一致。 */
	
}

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
// 无刷
//============================================================================
/*
 * 函数功能：解析 UART1 驱动板回包，更新实际转速、电流和驱动故障状态。
 * 输入参数：无，函数直接读取 UART1 DMA 接收缓冲区。
 * 返回参数：无；合法回包会更新 WorkMessage，并在故障时撤销电机和泵运行请求。
 */
void BrushlessMotorUartData_ReceiveData(void)
{
	static uint8_t clean_huic=0; /* 记录上一周期是否出现驱动故障，用于故障恢复边沿再执行一次安全全停。 */
  uint8_t rlen = 0, i = 0;
  uint8_t dat[UART1_MAX_PACKET_SIZE] = { 0 }, dat1[22] = { 0 };
  uint16_t CRC_Check_Vaule=0;
	
  //读取串口数据
  rlen = Uart1_DMARecvDataPeek(dat);
	/* 少于一帧所需字节时保留 DMA 数据等待后续接收，不能进入 CRC 和字段解析。 */
  if (rlen < 11)   //不够一个数据包大小
	  return;

  //查询本帧数据包的帧头0x01
  for (i = 0; i < (rlen - 11); i++)    //最小帧数据包为4[0x01 0x03 0 0 0 0 0]
  {
	  if (dat[i] == 0xAA)  
	  {
		  CRC_Check_Vaule = Common_Crc16(&dat[i],10);//ssc
			/* CRC 通过后才允许修改运行状态，避免串口噪声被误当成驱动故障或速度反馈。 */
			if(CRC_Check_Vaule==dat[i+10]+(dat[i+11]<<8))
			{
				Common_CopyData(&dat[i], dat1, 12);    //截取10个数据
				WorkMessage.driver_speed_feedback = (uint16_t)(((uint16_t)dat1[4] << 8U) | dat1[5]); /* 驱动 byte4~5 是实际转速反馈，单位沿用驱动私有协议的“转速/10”，只做监测不改目标速度。 */
				WorkMessage.driver_current_x100 = (uint16_t)(((uint16_t)dat1[8] << 8U) | dat1[9]);    /* 驱动 byte8~9 是 App.FB.Prot.AllCur * 100，单位 0.01A，只上传给上位机显示。 */
					/* byte7 为驱动故障码，0 表示本帧确认驱动已经恢复正常。 */
					if (dat1[7] == MOTOR_UART_DRIVER_ERR_NONE)
					{
						/* 只在“上一帧故障、本帧恢复”的边沿再次全停，防止旧运行请求随故障解除自动恢复输出。 */
						if(clean_huic==1){
							clean_huic=0;
							MotorUart_StopAllWork();
						}
						/* 仅清理由驱动故障创建的报警，不能覆盖手柄掉线等更高层报警。 */
						MotorUart_ClearDriverAlarmIfOwned();
						
					}
					else
					{
						/* 记录故障存在，供故障恢复帧执行一次安全全停。 */
						clean_huic=1;
						/* 把驱动私有故障码转换为主工程报警，保持蜂鸣、屏幕和上位机一致。 */
						MotorUart_SetDriverAlarm(dat1[7]);
						Pump_SetSpeed_A(0);//立即下发 A 泵零速，先切断当前泵硬件输出
						//Pump_SetSpeed_B(0);//泵停止运行
						pumpMessageA.run_flag = false; /* 撤销 A 泵运行请求，防止泵任务下一周期重新拉起。 */
						//pumpMessageA.speed_work = 0U;
						pumpMessageB.run_flag = false; /* 同步撤销 B 泵请求，使驱动故障进入全泵停机状态。 */
						//pumpMessageB.speed_work = 0U;
						// MotorUart_StopAllWork();
					}	
					/* byte1 是驱动当前方向回显；现阶段仅保留协议分支，不据此改写主控方向状态。 */
					switch (dat1[1])
				{
					case 0x01 :  //正向
					{

					}
					break;
					case 0x02 :  //反向HALLEErrFlag
					{

					}
					break;
					case 0x03 :  //往复
					{

					}
					break;
					case 0x04 :
					case 0x05 :
					{
					}
					break;
					default : break;
				}

				memset(dat1, 0, 15);
				i += 12;
			}
	  }
  }
}

//============================================================================
//与主板进行串口通讯的任务初始化 2
//============================================================================
/* USER CODE BEGIN Header_MOTORUARTTaskFunc */
/**
* @brief Function implementing the MOTORUARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTORUARTTaskFunc */
void MOTORUARTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN MOTORUARTTaskFunc */
  /*
   * UART2 已由 ExternalComm 独立任务接管，用于新的外部通信协议。
   * 本任务只保留 UART1 驱动板接收，避免两个任务同时读取 UART2 DMA 缓冲。
   */
  BrushlessMotorUartData_ReceiveData();
  /* USER CODE END MOTORUARTTaskFunc */
}

void MotorUartData_Init(void)
{
  /* definition and creation of MOTORUARTTask */
	Kernel_TaskCreate(&MOTORUARTTaskHandle, MOTORUARTTaskFunc);
	Kernel_TaskStart(&MOTORUARTTaskHandle, KERNEL_TASK_ALWAYS, 3);//3
}





