//adc.c

#include "stm32f4xx_hal.h"
#include "adc.h"
#include "delay.h"

extern ADC_HandleTypeDef hadc1;

static uint16_t ADCConvertedValue[ADC_CHANNEL_CNT] = { 0 };

//最快转换2.4MHz即0.41us
void Adc_StartInit(void)
{
//  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADCConvertedValue[0], ADC_CHANNEL_CNT);
}


uint16_t Adc_GetSingleData(uint8_t chn)
{
  return  ADCConvertedValue[chn];
}


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




















