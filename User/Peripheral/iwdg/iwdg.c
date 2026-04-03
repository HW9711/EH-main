//iwdg.c

#include "stm32f4xx_hal.h"
#include "iwdg.h"

#include "app_task.h"

extern IWDG_HandleTypeDef hiwdg;

task_t IWDGTaskHandle;

void Iwdg_Init(void)
{

}


void Iwdg_Reset(void)
{
//  HAL_IWDG_Refresh(&hiwdg);
}

//============================================================================
void IwdgTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN IWDGTaskFunc */
  /* Infinite loop */
	  Iwdg_Reset();
  /* USER CODE END IWDGTaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 300
 */
//============================================================================
void IwdgTaskInit(void)
{
  /* definition and creation of IWDGTask */
  app_task_create(&IWDGTaskHandle, IwdgTaskFunc);
  app_task_start(&IWDGTaskHandle, APP_TASK_ALWAYS, 300);
}
