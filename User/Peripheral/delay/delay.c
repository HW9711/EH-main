//delay.c
//阻塞延时(ms)...

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "iwdg.h"

extern TIM_HandleTypeDef htim7;

void Delay_us(uint16_t cnt)
{
  uint16_t i = 0;

  htim7.Instance->CNT = 0;

  while(i < cnt)
  {
    i = htim7.Instance->CNT;
  }
}

void Delay_ms(uint16_t cnt)
{
  uint16_t i;
  for(i = 0; i < cnt; i++)
  {
    Iwdg_Reset();
    Delay_us(990);
  }
}



