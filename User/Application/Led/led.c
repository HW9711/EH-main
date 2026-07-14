//led.c

#include "stm32f4xx_hal.h"
#include "led.h"
#include "board.h"

/*
 * 函数功能：按手柄状态编号点亮对应的一路指示灯，并关闭其它三路指示灯。
 * 输入参数：st 为指示灯状态，1~4 分别选择 H1~H4，其它值表示全部关闭。
 * 返回参数：无。
 */
void Led_HandleState(uint8_t st)
{
	static uint8_t stateLast = 0xff; /* 缓存上次状态，避免周期任务重复写同一组 GPIO。 */

	/* 状态没有变化时保持现有硬件输出，减少无意义的 GPIO 翻转。 */
	if (st == stateLast)
		return ;

	/* 每个有效状态只点亮对应一路，默认分支用于离线或未知状态时全部熄灭。 */
	switch (st)
	{
		case 1 :
		{
			/* 状态 1 只点亮 H1，确保其它通道指示灯不会保留旧状态。 */
			LED_H2_OFF();
			LED_H3_OFF();
			LED_H4_OFF();

			LED_H1_ON();
		}
		break;
		case 2 :
		{
			/* 状态 2 只点亮 H2，用于明确显示第二路状态。 */
			LED_H1_OFF();
			LED_H3_OFF();
			LED_H4_OFF();

			LED_H2_ON();
		}
		break;
		case 3 :
		{
			/* 状态 3 只点亮 H3，用于明确显示第三路状态。 */
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H4_OFF();

			LED_H3_ON();
		}
		break;
		case 4 :
		{
			/* 状态 4 只点亮 H4，用于明确显示第四路状态。 */
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H3_OFF();

			LED_H4_ON();
		}
		break;
		default :
		{
			/* 未知或离线状态关闭全部指示灯，避免错误状态被误认为有效通道。 */
			LED_H1_OFF();
			LED_H2_OFF();
			LED_H3_OFF();
			LED_H4_OFF();
		}
		break;
	}

	stateLast = st;
}
	





