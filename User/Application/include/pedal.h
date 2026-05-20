//pedal.h

#ifndef __PEDAL_H
#define __PEDAL_H

#include <stdint.h>

typedef struct
{
  uint16_t FootPedalADValue;              // 单踏板或左踏板实时 AD 值，定标页用于显示当前踩踏量。
  uint16_t FootPedalMemoryLValue;         // 单踏板低点定标值，脚踏板回包后刷新。
  uint16_t FootPedalMemoryHValue;         // 单踏板高点定标值，脚踏板回包后刷新。
  uint16_t FootPedalADValue_Right;        // 右踏板实时 AD 值，双踏板定标页用于显示。
  uint16_t FootPedalADValue_Left;         // 左踏板实时 AD 值，双踏板定标页用于显示。
  uint16_t FootPedalMemoryLValue_Right;   // 右踏板低点定标值。
  uint16_t FootPedalMemoryLValue_Left;    // 左踏板低点定标值。
  uint16_t FootPedalMemoryHValue_Right;   // 右踏板高点定标值。
  uint16_t FootPedalMemoryHValue_Left;    // 左踏板高点定标值。
  uint16_t FootPedalMemoryMValue_Right;   // 右踏板中点定标值。
  uint16_t FootPedalMemoryMValue_Left;    // 左踏板中点定标值。
  uint8_t FootPedalType;                  // 脚踏类型：0=单踏板，1=双踏板，定标页按此决定左右值显示。
  uint8_t FootPedalConnectFlag;           // 最近一次定标串口回包连接标志，保留给定标流程判断。
  uint8_t FootPedalOffTimes;              // 定标串口离线计数，保持原接收流程的断线计时入口。
  uint8_t FootPedalKeyValue;              // 脚踏按键原始编号，调试或后续联动时可读取。
} PedalCalibrationData_t;

extern PedalCalibrationData_t PedalCalibrationData;

void Pedal_StorageHValue(void);  //JT_Memory_H
void Pedal_StorageLValue(void);  //JT_Memory_L
void Pedal_StorageMValue(void);  //JT_Memory_M

void Pedal_StorageHValue_Left(void);  //JT_Memory_H
void Pedal_StorageLValue_Left(void);  //JT_Memory_L
void Pedal_StorageMValue_Left(void);  //JT_Memory_M 

void Pedal_ReadHValue(void);  //JT_Read_H
void Pedal_ReadHValue_Left(void);

void Pedal_ReadLValue(void);  //JT_Read_L
void Pedal_ReadLValue_Left(void);
	
void Pedal_ReadMValue(void);	
void Pedal_ReadMValue_Left(void);	
	
void Pedal_SendState(uint8_t State, uint8_t Type);  //JT_Read_State

void Pedal_SendStateWarn(uint8_t State, uint8_t Type);  //JT_Read_StateWarn

void Pedal_ReadStorageHLValue(void);  //Read_JT_Memory

void PedalRecv_Scan(void);

void PedalRecvTask_Init(void);

#endif  //__PEDAL_H


