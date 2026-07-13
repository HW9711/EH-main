//led.c

#include "stm32f4xx_hal.h"
#include "led.h"
#include "board.h"

void Led_HandleState(uint8_t st)
{
	static uint8_t stateLast = 0xff;

	if (st == stateLast)
		return ;

	switch (st)
	{
		case 1 :
		{
			LED_H2_OFF();
			LED_H3_OFF();
			LED_H4_OFF();

			LED_H1_ON();
		}
		break;
		case 2 :
		{
			LED_H1_OFF();
			LED_H3_OFF();
			LED_H4_OFF();

			LED_H2_ON();
		}
		break;
		case 3 :
		{
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H4_OFF();

			LED_H3_ON();
		}
		break;
		case 4 :
		{
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H3_OFF();

			LED_H4_ON();
		}
		break;
		default :
		{
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H3_OFF();
			LED_H4_OFF();
		}
		break;
	}

	stateLast = st;
}
	





