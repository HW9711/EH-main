//drivectrl.h

#ifndef __DRIVECTRL_H
#define __DRIVECTRL_H

void DriveCtrl_PumpFlag_A(void); /* 清除 A 泵运行和排空状态，并立即发送零速。 */

//============================================================================
void DriveCtrl_PumpFlag_B(void); /* 清除 B 泵运行和排空状态，并立即发送零速。 */

//============================================================================
void DriveCtrl_Motor1CurrentTask_Init(void); /* 旧任务声明，当前驱动控制文件未提供实现。 */

//============================================================================
void DriveCtrl_UIRefreshDataTask_Init(void); /* 旧任务声明，当前驱动控制文件未提供实现。 */

//============================================================================
void DriveCtrl_Motor123Task_Init(void); /* 旧任务声明；当前电机周期输出由 SscDriveMotorTask_Init 创建。 */

void DriveCtrl_HMITask_Init(void); /* 旧外控任务声明；当前外控由 ExternalComm 模块处理。 */

#endif  //__DRIVECTRL_H


