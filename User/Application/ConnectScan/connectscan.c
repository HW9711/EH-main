//connectscan.c

#include "connectscan.h"
#include "data.h"

#include <stdint.h>

#include "kernel_scheduler.h"

kernel_task_t CONNECTSCANTaskHandle;

//============================================================================
// 函数名称: Connectscan_Fun()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void Connectscan_Fun(void)
{
  ////////////////驱动板扫描 1s////////////////////
  if (++SysRunData.DriveBoardOffTimes >= 50)
  {
	  SysRunData.DriveBoardOffTimes = 0;

	  SysRunData.DriveBoardConnectFlag = No_Connect;
  }

  ///////////////检测脚踏是否连接/////////////////
  if(++SysFootPedalData.FootPedalOffTimes >= 20)  //延时400ms检测脚踏是否连接
  {
	  SysFootPedalData.FootPedalOffTimes = 0;

	  SysFootPedalData.FootPedalConnectFlag = No_Connect; //脚踏没有连接

	  SysFootPedalData.FootPedalMemoryLValue = 0;
	  SysFootPedalData.FootPedalMemoryHValue = 0;

	  SysFootPedalData.FootPedalADValue = 0;  //防止在电机转动时，脚踏断开，电机仍在运行
  }
}

/* USER CODE BEGIN Header_CONNECTSCANTaskFunc */
/**
* @brief Function implementing the CONNECTSCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONNECTSCANTaskFunc */
void CONNECTSCANTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN CONNECTSCANTaskFunc */
  /* Infinite loop */
  Connectscan_Fun();
  /* USER CODE END CONNECTSCANTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 20
 */
//============================================================================
void ConnectscanTaskInit(void)
{
  /* definition and creation of BEEPTask */
  Kernel_TaskCreate(&CONNECTSCANTaskHandle, CONNECTSCANTaskFunc);
  Kernel_TaskStart(&CONNECTSCANTaskHandle, KERNEL_TASK_ALWAYS, 20);
}






