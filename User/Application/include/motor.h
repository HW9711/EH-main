//motor.h

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

//============================================================================
// 函数名称: BrushedMotor_Stop()
// 功能描述: 发送停止数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
// 输    出: 无
// 函数说明:
//============================================================================
void BrushedMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq);

//============================================================================
// 函数名称: BrushedMotor_Run()
// 功能描述: 发送运行数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Speed：转速
//           Current：电流
// 输    出: 无
// 函数说明:
//============================================================================
void BrushedMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint16_t Speed, uint16_t Current);

//============================================================================
// 函数名称: BrushlessMotor_Stop()
// 功能描述: 发送停止数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Mode1：闭环方式 0x02 方波霍尔
// 输    出: 无
// 函数说明:
//============================================================================
void BrushlessMotor_Stop(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1);

//============================================================================
// 函数名称: BrushlessMotor_Run()
// 功能描述: 发送运行数据
// 输　  入: MotorNum：电机号 1、2
//           Mode：1正，2反，3往复
//           Freq：往复频率（仅mode是3时有效）
//           Mode1：闭环方式 0x02 方波霍尔
//           Speed：转速
// 输    出: 无
// 函数说明:
//============================================================================
void BrushlessMotor_Run(uint8_t MotorNum, uint8_t Mode, uint8_t Freq, uint8_t Mode1, uint16_t Speed, uint16_t Current);

//============================================================================
// 函数名称: BrushlessMotor_SetPosition()
// 功能描述: 设置电机位置
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void BrushlessMotor_SetPosition(uint8_t MotorNum, uint8_t Type, uint16_t NUM);

//============================================================================
// 函数名称: Motor_ErrorEmergencyStop_Ctrl()
// 功能描述: 电机急停
// 输　  入:
// 输    出:
// 函数说明: 
//============================================================================
void Motor_ErrorEmergencyStop_Ctrl(void);

#endif  //__MOTOR_H


