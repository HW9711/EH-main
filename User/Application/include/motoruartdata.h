//motoruartdata.h

#ifndef __MOTORUARTDATA_H
#define __MOTORUARTDATA_H

void MotorUartData_Init(void);

/*
 * 函数功能：通知电机反馈模块，过载/堵转发生后的脚踏已经真实松开。
 * 输入参数：无。
 * 返回参数：无；驱动已恢复时立即清除脚踏过载报警，否则等待后续 Err=0 再清除。
 */
void MotorUart_ReleaseFootOverload(void);

#endif  //__MOTORUARTDATA_H



