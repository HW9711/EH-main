//pedal.h

#ifndef __PEDAL_H
#define __PEDAL_H

#include <stdint.h>

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


