//motoruartdata.h

#ifndef __MOTORUARTDATA_H
#define __MOTORUARTDATA_H

/*
 * 电机驱动细分报警图片开关：
 * 0U：生产模式只显示 84/86/87 公用报警图，未配置公用图的驱动错误不弹出误导图片。
 * 1U：内部调试模式按驱动原始 Err 显示 91~99 细分报警图。
 * 该开关只影响屏幕图片，不改变停机、蜂鸣、脚踏过载锁存和外控报警码。
 */
#ifndef MOTOR_ALARM_DETAIL_ENABLE
#define MOTOR_ALARM_DETAIL_ENABLE 1U
#endif

/* 驱动错误没有可用屏幕图片时使用该保留值，显示任务收到后关闭旧报警图而不显示错误文案。 */
#define MOTOR_ALARM_PICTURE_NONE 0xFFU

void MotorUartData_Init(void);

/*
 * 函数功能：通知电机反馈模块，过载/堵转发生后的脚踏已经真实松开。
 * 输入参数：无。
 * 返回参数：无；驱动已恢复时立即清除脚踏过载报警，否则等待后续 Err=0 再清除。
 */
void MotorUart_ReleaseFootOverload(void);

#endif  //__MOTORUARTDATA_H



