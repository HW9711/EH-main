//adc.c

#include "stm32f4xx_hal.h"
#include "adc.h"
#include "delay.h"

extern ADC_HandleTypeDef hadc1;

static uint16_t ADCConvertedValue[ADC_CHANNEL_CNT] = { 0 };

/*
 * 函数功能：启动ADC和DMA，让采样结果持续写入ADCConvertedValue；速率由ADC初始化参数决定。
 * 输入参数：无。
 * 返回参数：无；此处未检查HAL启动结果。
 */
void Adc_StartInit(void)
{
//  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADCConvertedValue[0], ADC_CHANNEL_CNT);
}


uint16_t Adc_GetSingleData(uint8_t chn)
{
  return  ADCConvertedValue[chn];
}


/*
 * 函数功能：每隔20us读取一次指定通道的DMA结果，累加后取平均。
 * 输入参数：chn为数组下标，须小于ADC_CHANNEL_CNT；cnt为取样次数，不能为0。
 * 返回参数：平均ADC原始值，未经电压或物理量换算；DMA未更新时可能多次读到同一结果。
 */
uint16_t Adc_GetData(uint8_t chn, uint8_t cnt)
{
  uint32_t temp_val = 0;
  uint8_t i = 0;

  for(i = 0; i < cnt; i++)
  {
	  temp_val += ADCConvertedValue[chn];
	  Delay_us(20);
  }

  return temp_val / cnt;
}




















