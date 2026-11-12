//motoruartdata.h

#ifndef __MOTORUARTDATA_H
#define __MOTORUARTDATA_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PROTECTION_FRAME_ADDRESS 0xABU /* 新运行协议地址；旧0xAA驱动不会把扩展字段误解释为运动命令。 */
#define MOTOR_PROTECTION_COMMAND_LEN 15U /* byte9~12为参数22/24大端毫秒，byte13~14为CRC。 */
#define MOTOR_PROTECTION_FEEDBACK_LEN 16U /* byte10~13回报实际时间，byte14~15为CRC。 */
#define MOTOR_PROTECTION_FEEDBACK_TIMEOUT_MS 200U /* 新协议能力证明只在最近200ms内有效，失联后必须重新确认。 */
uint8_t MotorUart_IsProtectionProtocolReady(void); /* 仅有效新协议反馈允许主控发出运动命令。 */

/* 只保存通过CRC检查的完整12/16字节回包，各字段必须来自同一包。 */
typedef struct
{
	uint32_t feedback_tick_ms; /* HAL毫秒时钟：最近一次有效驱动回包形成快照的时刻。 */
	uint32_t rx_window_start_ms; /* 该帧所属DMA接收窗口的起点，用于排除停机前缓存的反馈。 */
	uint32_t speed_rpm;        /* 驱动 byte4~5 的“转速/10”字段换算成实际 rpm。 */
	uint16_t sequence;         /* 每份CRC正确回包递增一次，自然回绕。 */
	uint16_t current_x100;     /* 驱动 byte8~9 实时工作电流，单位0.01A。 */
	uint16_t overcurrent_time22_ms; /* 新反馈中的实际有霍尔往复时间；旧协议为0，不代表驱动采用值。 */
	uint16_t overcurrent_time24_ms; /* 新反馈中的实际无霍尔时间，单位ms。 */
	uint8_t protection_protocol_valid; /* 1表示新协议且两项时间均合法，旧回复不能授予启动权限。 */
	uint8_t motor_kind; /* 实际反馈物理通道与电机类型，1/2无刷、3/4有刷。 */
	uint8_t raw_error;         /* 驱动 byte7 原始错误码，不经过主控报警映射。 */
	uint8_t valid;             /* 已收到至少一份CRC正确回包时置1。 */
} MotorUartFeedbackSnapshot_t;

/* 驱动速度反馈的有效时间，单位 ms。停止请求发出后，超过 250ms 的旧非零速度不再阻止切换控制方式。
 * 调大后切换可能等待更久；调小后会更早忽略旧反馈。超时不等于已经实测停转，也不清除记录值。
 */
#define MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS 250U

/*
 * 电机驱动扩展故障图片映射开关：
 * 0U：只使用 84/86/87 公用报警图；没有对应图片的错误不显示图片。
 * 1U：原91~99细分报警统一显示87号“电机过载，请检查刀具是否卡死”；缺相仍显示84。当前默认开启。
 * 该开关只影响屏幕图片，不改变停机、蜂鸣、脚踏过载锁存和外控报警码。
 */
#ifndef MOTOR_ALARM_DETAIL_ENABLE
#define MOTOR_ALARM_DETAIL_ENABLE 1U
#endif

/* 驱动错误没有可用屏幕图片时使用该保留值，显示任务收到后关闭旧报警图而不显示错误文案。 */
#define MOTOR_ALARM_PICTURE_NONE 0xFFU

/* 驱动参数维护回包最大为：地址、功能码、字节数、30个16位参数和CRC，共65字节。 */
#define MOTOR_UART_DRIVER_PARAMETER_MAX_FRAME_SIZE 65U

/* 参数读写结果只说明 UART1 是否收到响应；参数是否有效还要查看驱动板返回的内容。 */
typedef enum
{
	MOTOR_UART_DRIVER_PARAMETER_RESULT_NONE = 0U, /* 当前没有可取走的维护事务结果。 */
	MOTOR_UART_DRIVER_PARAMETER_RESULT_OK,        /* 已收到CRC正确的0xFD维护响应。 */
	MOTOR_UART_DRIVER_PARAMETER_RESULT_TIMEOUT    /* 300ms内没有收到完整维护响应。 */
} MotorUartDriverParameterResult_t;

void MotorUartData_Init(void);

/*
 * 函数功能：上报STOP发送失败并撤销业务运行，等待后续周期成功补发零速。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorUart_NotifyStopTransmitFailure(void);

/*
 * 函数功能：读取最近一次有效驱动反馈，读取途中发生更新时重试，避免混用新旧数据。
 * 输入参数：snapshot指向接收反馈记录的结构体，复制后调用方读取自己的这份数据。
 * 返回参数：已有有效反馈记录且复制期间未被修改时返回1，否则返回0。
 */
uint8_t MotorUart_CopyFeedbackSnapshot(MotorUartFeedbackSnapshot_t *snapshot);

/*
 * 函数功能：判断近期有效驱动回包是否仍报告电机转动，过期回包只保留诊断值、不再作为忙状态。
 * 输入参数：无，反馈超过 MOTOR_UART_FEEDBACK_MOTION_TIMEOUT_MS 毫秒后不再用于判断是否转动。
 * 返回参数：true表示近期反馈速度非零，复制冲突时按旧非零值保守保持；false表示零速、尚无反馈或反馈已过期。
 */
bool MotorUart_IsRecentFeedbackMoving(void);

/*
 * 函数功能：查询当前是否有脚踏来源的驱动故障仍在等待脚踏释放。
 * 输入参数：无。
 * 返回参数：true表示脚踏任务必须继续检测释放；false表示不是脚踏来源或释放已经记录。
 */
bool MotorUart_IsFootDriverAlarmWaitingRelease(void);

/*
 * 函数功能：查询当前驱动报警是否在电机运行期间开始且仍由驱动反馈模块持有。
 * 输入参数：无。
 * 返回参数：true表示物理拔柄应按运行掉线处理；false表示没有可接管的运行驱动报警。
 */
bool MotorUart_DidDriverAlarmStartDuringRun(void);

/*
 * 函数功能：通知电机反馈模块，脚踏来源驱动故障已经观察到松脚、掉线或实时数据失效。
 * 输入参数：无。
 * 返回参数：无；驱动已恢复时立即清除报警，否则保存释放结果并等待后续Err=0。
 */
void MotorUart_ReleaseFootDriverAlarm(void);

/*
 * 函数功能：发送一次驱动参数读写请求；等待响应和取走结果期间暂停周期 0xAA 控制帧。
 * 输入参数：request指向完整8字节Modbus请求；request_len必须为8。
 * 返回参数：事务成功启动返回1；参数错误或已有事务占用返回0。
 */
uint8_t MotorUart_StartDriverParameterRequest(const uint8_t *request, uint8_t request_len);

/*
 * 函数功能：取走驱动参数读写的响应或超时结果，让 UART1 恢复发送周期控制帧。
 * 输入参数：response接收原始驱动回包；response_len接收长度；result接收链路结果。
 * 返回参数：存在可取结果返回1；事务仍在等待或当前空闲返回0。
 */
uint8_t MotorUart_PollDriverParameterResponse(uint8_t *response,
													 uint8_t *response_len,
													 MotorUartDriverParameterResult_t *result);

/*
 * 函数功能：查询 UART1 是否正在等待参数响应，或等待上层取走读写结果。
 * 输入参数：无。
 * 返回参数：等待或结果待取时返回1；空闲时返回0。
 */
uint8_t MotorUart_IsDriverParameterTransactionActive(void);

/*
 * 函数功能：停机时结束当前参数读写等待，让 UART1 可以发送零速停止帧。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorUart_AbortDriverParameterTransaction(void);

#endif  //__MOTORUARTDATA_H



