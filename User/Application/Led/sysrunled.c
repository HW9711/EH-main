//sysrunled.c

#include "stm32f4xx_hal.h"
#include "sysrunled.h"

#include "app_task.h"

task_t LEDTaskHandle;

/* USER CODE BEGIN Header_LEDTaskFunc */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTaskFunc */
void LEDTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN LEDTaskFunc */
  /* Infinite loop */
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
  /* USER CODE END LEDTaskFunc */
}

void LEDTaskInit(void)
{
  /* definition and creation of LEDTask */
  app_task_create(&LEDTaskHandle, LEDTaskFunc);
  app_task_start(&LEDTaskHandle, APP_TASK_ALWAYS, 200);
}








