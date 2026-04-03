//drivectrl.h

#ifndef __DRIVECTRL_H
#define __DRIVECTRL_H

void DriveCtrl_PumpFlag_A(void);

//============================================================================
void DriveCtrl_PumpFlag_B(void);

//============================================================================
void DriveCtrl_Motor1CurrentTask_Init(void);

//============================================================================
void DriveCtrl_UIRefreshDataTask_Init(void);

//============================================================================
void DriveCtrl_Motor123Task_Init(void);

void DriveCtrl_HMITask_Init(void);

#endif  //__DRIVECTRL_H


