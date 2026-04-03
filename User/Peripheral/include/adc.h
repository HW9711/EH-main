//adc.h

#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>


#define ADC_CHANNEL_CNT  4

//----------------- -----------------
#define T_CHANNEL   	      4
#define H_KEY1_CHANNEL		  3
#define H_KEY2_CHANNEL   	  1
#define V_CHANNEL   	      2

void Adc_StartInit(void);
uint16_t Adc_GetSingleData(uint8_t chn);
uint16_t Adc_GetData(uint8_t chn, uint8_t times);

#endif  //__ADC_H__


