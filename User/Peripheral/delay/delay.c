//delay.c
//使用TIM7计数等待；等待期间函数不返回，不是操作系统的任务休眠。

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "iwdg.h"

extern TIM_HandleTypeDef htim7;

/*
 * 函数功能：清零TIM7计数器后等待cnt个计数，按当前每计数1us的配置提供微秒延时。
 * 输入参数：cnt为要等待的微秒数；TIM7须已启动且计数周期设置正确。
 * 返回参数：无。
 */
void Delay_us(uint16_t cnt)
{
  uint16_t i = 0;

  htim7.Instance->CNT = 0;

  while(i < cnt)
  {
    i = htim7.Instance->CNT;
  }
}

/*
 * 函数功能：重复执行喂狗和990us等待，得到近似毫秒延时。
 * 输入参数：cnt为重复次数，约等于毫秒数；实际时间还包含调用开销和中断耗时。
 * 返回参数：无。
 */
void Delay_ms(uint16_t cnt)
{
  uint16_t i;
  for(i = 0; i < cnt; i++)
  {
    Iwdg_Reset();
    Delay_us(990);
  }
}



