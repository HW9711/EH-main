//dircurrent.c

#include "dircurrent.h"

#include "data.h"

#include "kernel_scheduler.h"

kernel_task_t DIRCURRENTTaskHandle;

//============================================================================
//峰岹驱动电流判断
//============================================================================
void DirCurrentTask_Judge_Fun(void)
{
  static	uint8_t CurrentTiming2s = 0;
  static	uint8_t MotorCurrentTiming = 0;

  static	uint8_t CurrentTiming200ms = 0;

  if ((SysRunData.MotorRun != Pedal_Run) || (SysRunData.MotorNum != MotorNum1))
  {
	  SysRunData.DirCurrent = 0;
		CurrentTiming2s = 0;
	  MotorCurrentTiming = 0;
	  CurrentTiming200ms = 0;
		SysRunData.StuckFlag = No_Error;

	  return ;
  }

  switch (SysRunData.MotorType)
  {
	  case MOTORTYPE_EC13 :
	  {
	    if ((SysRunData.DirCurrent >= 450) && (SysRunData.DirCurrent < 550))
	    {
		    CurrentTiming200ms = 0;

		    if (++CurrentTiming2s >= 200)  //2s
		    {
		      CurrentTiming2s = 0;

		      SysRunData.StuckFlag = Error;
		      SysRunData.WarnID = 5;
		    }
	    }
	    else if (SysRunData.DirCurrent >= 550)  //Max_Motor_Current
	    {
		    CurrentTiming200ms = 0;

		    if (++MotorCurrentTiming >= 2) //210ms
		    {
		      MotorCurrentTiming = 0;

		      SysRunData.StuckFlag = Error;
		      SysRunData.WarnID = 5;
		    }
	    }
//			else if(SysRunData.DirCurrent >= 600)  //Max_Motor_Current
//			{
//				CurrentTiming200ms = 0;

//				SysRunData.StuckFlag = Error;

//				SysRunData.WarnID = 5;
//			}
	    else
	    {
		    if (++CurrentTiming200ms >= 5)
		    {
		      CurrentTiming200ms = 0;
		      MotorCurrentTiming = 0;
		      CurrentTiming2s = 0;
		    }
	    }
	  }
	  break;
	  case MOTORTYPE_EC16 :
	  {
	    if ((SysRunData.DirCurrent >= 600) && (SysRunData.DirCurrent < 800))
	    {
		    CurrentTiming200ms = 0;

		    if (++CurrentTiming2s >= 200)  //2s
		    {
		      CurrentTiming2s = 0;

		      SysRunData.StuckFlag = Error;
  		    SysRunData.WarnID = 5;
		    }
	    }
	    else if (SysRunData.DirCurrent >= 800)  //Max_Motor_Current
	    {
		    CurrentTiming200ms = 0;

		    if (++MotorCurrentTiming >= 2) //210ms
		    {
		      MotorCurrentTiming = 0;

		      SysRunData.StuckFlag = Error;
  		    SysRunData.WarnID = 5;
		    }
	    }
	    else
	    {
		    if (++CurrentTiming200ms >= 5)
		    {
		      CurrentTiming200ms = 0;
		      MotorCurrentTiming = 0;
		      CurrentTiming2s = 0;
		    }
	    }
	  }
	  break;
	  default : break;
  }
}

//============================================================================
// 13
//============================================================================
/* USER CODE BEGIN Header_DIRCURRENTTaskFunc */
/**
* @brief Function implementing the DIRCURRENTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DIRCURRENTTaskFunc */
void DIRCURRENTTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN DIRCURRENTTaskFunc */
  /* Infinite loop */
  DirCurrentTask_Judge_Fun();
  /* USER CODE END DIRCURRENTTaskFunc */
}

void DirCurrentTask_Judge_Init(void)
{
  /* definition and creation of DIRCURRENTTask */
	Kernel_TaskCreate(&DIRCURRENTTaskHandle, DIRCURRENTTaskFunc);
	Kernel_TaskStart(&DIRCURRENTTaskHandle, KERNEL_TASK_ALWAYS, 13);
}










