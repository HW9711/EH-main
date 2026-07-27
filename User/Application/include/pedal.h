//pedal.h

#ifndef __PEDAL_H
#define __PEDAL_H

#include <stdint.h>

#define PEDAL_CAL_TYPE_NONE       0U  // 尚未收到可识别的脚踏实时数据，定标页保持空闲显示。
#define PEDAL_CAL_TYPE_SINGLE     1U  // 单踏板使用左侧低点和高点两点定标。
#define PEDAL_CAL_TYPE_TWO_STAGE  2U  // 双段踏板使用左侧低点、中点和高点三点定标。
#define PEDAL_CAL_TYPE_DUAL       3U  // 双脚踏分别保存左右两组三点定标值。

typedef struct
{
  uint16_t FootPedalADValue_Left;       // 单踏板、双段踏板或双脚踏左路的实时 AD 值。
  uint16_t FootPedalADValue_Right;      // 双脚踏右路的实时 AD 值，非双脚踏时保持为零。
  uint16_t FootPedalMemoryLValue_Left;  // 左路或单路低点定标值。
  uint16_t FootPedalMemoryMValue_Left;  // 左路中点定标值，单踏板不使用。
  uint16_t FootPedalMemoryHValue_Left;  // 左路或单路高点定标值。
  uint16_t FootPedalMemoryLValue_Right; // 双脚踏右路低点定标值。
  uint16_t FootPedalMemoryMValue_Right; // 双脚踏右路中点定标值。
  uint16_t FootPedalMemoryHValue_Right; // 双脚踏右路高点定标值。
  uint16_t FootPedalOffTimes;           // 标定模式下连续未收到合法帧的2ms扫描次数。
  uint8_t FootPedalType;                // 当前识别出的脚踏类型，使用 PEDAL_CAL_TYPE_* 取值。
  uint8_t FootPedalConnectFlag;         // 合法脚踏帧到达后置1，约1秒无合法帧后清零。
  uint8_t FootPedalKeyValue;            // 最近一次脚踏实体按键的规范化编号：1左、2中、3右。
} PedalCalibrationData_t;

extern PedalCalibrationData_t PedalCalibrationData;

void PedalCal_Reset(void);

void Pedal_StorageHValue(void);        // 保存单路或左路高点值。
void Pedal_StorageLValue(void);        // 保存单路或左路低点值。
void Pedal_StorageMValue(void);        // 保存左路中点值。
void Pedal_StorageHValue_Right(void);  // 保存双脚踏右路高点值。
void Pedal_StorageLValue_Right(void);  // 保存双脚踏右路低点值。
void Pedal_StorageMValue_Right(void);  // 保存双脚踏右路中点值。

void Pedal_ReadHValue(void);        // 请求脚踏板重新加载高点存储值。
void Pedal_ReadLValue(void);        // 请求脚踏板重新加载低点存储值。
void Pedal_ReadMValue(void);        // 请求脚踏板重新加载中点存储值。
void Pedal_ReadHValue_Right(void);  // 兼容旧双脚踏协议的右路高点读取命令。
void Pedal_ReadLValue_Right(void);  // 兼容旧双脚踏协议的右路低点读取命令。
void Pedal_ReadMValue_Right(void);  // 兼容旧双脚踏协议的右路中点读取命令。

void Pedal_SendState(uint8_t State, uint8_t Type);
void Pedal_SendStateWarn(uint8_t State, uint8_t Type);
void Pedal_ReadStorageHLValue(void);
void PedalRecv_Scan(void);
void PedalRecvTask_Init(void);
#endif  //__PEDAL_H


