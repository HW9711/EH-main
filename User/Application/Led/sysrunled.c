//sysrunled.c

#include "bsp_gpio.h"
#include "sysrunled.h"

#include "kernel_scheduler.h"

kernel_task_t LEDTaskHandle;

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
  Bsp_GpioToggle(BOARD_RES_STATE_LED_PORT, BOARD_RES_STATE_LED_PIN);
  /* USER CODE END LEDTaskFunc */
}

void LEDTaskInit(void)
{
  /* definition and creation of LEDTask */
  Kernel_TaskCreate(&LEDTaskHandle, LEDTaskFunc);
  Kernel_TaskStart(&LEDTaskHandle, KERNEL_TASK_ALWAYS, 200);
}








