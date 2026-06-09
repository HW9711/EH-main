#include "stm32f4xx_hal.h"
#include "datahand.h"
UIDataShow UIDataShow_s;
UIControlState UIControlState_s;

//切换手柄时候初始化数据
void SscUIDataInit()
{
	static uint8_t handel_type=0;
	switch(handel_type)
	{
		case 1:                            //tmba
			break;
		case 2:                            //tmbb
			break;
		case 3:                            //emba
			break;
		case 4:                            //embb
		break;
	} 
}

//UI更新静态数据
void SscDataUpdate()
{
	
}


