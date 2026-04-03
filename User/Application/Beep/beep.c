//beep.c

#include "stm32f4xx_hal.h"
#include "beep.h"
#include "board.h"
#include "data.h"

#include "delay.h"

//============================================================================
// 函数名称: Beep_OnOffCtrl()
// 功能描述:
// 输　  入: 1开 0关
// 输    出:
// 函数说明:
//============================================================================
void Beep_OnOffCtrl(uint8_t OnOff)
{
  static uint8_t OnOffFlagLast = 0x55;

  if (OnOff == OnOffFlagLast)
	  return ;

  OnOffFlagLast = OnOff;

  if (OnOff)
	  BEEP_ON();
  else
	  BEEP_OFF();
}

//============================================================================
// 函数名称: Beep_BlockRun()
// 功能描述: 阻塞响蜂鸣器
// 输　  入:
// 输    出:
// 函数说明: 
//============================================================================
void Beep_BlockRun(uint8_t cnt, uint8_t ONt)
{
	uint8_t i = 0;
	for (i = 0; i < cnt; i++)
	{
	  Beep_OnOffCtrl(1);
	  Delay_ms(ONt);
	  Beep_OnOffCtrl(0);

		if (i < (cnt - 1))
			Delay_ms(ONt);
	}
}

//============================================================================
// 函数名称: Beep_RunStatus()
// 功能描述:
// 输　  入:
// 输    出:
// 函数说明: 10ms 中断
//============================================================================
void Beep_RunStatus(void)
{
  static uint8_t W_BEEP_Time = 0;
  static uint8_t W_BEEP_Time1 = 0;

  ////////////////蜂鸣器////////////////////
  //1.状态报警
  if (SysRunData.WarningBeep == 1)  //响四声 130ms
  {
	  W_BEEP_Time++;
		
	  switch (W_BEEP_Time)
	  {
	    case 1 :
	    case 27 :
	    case 53 :
	    case 79 :
		    Beep_OnOffCtrl(1);
	    break;
	    case 14 :
	    case 40 :
	    case 66 :
	    case 92 :
		    Beep_OnOffCtrl(0);
	    break;
	    case 130 : W_BEEP_Time = 0; break;
	    default : break;
	  }

    SysRunData.WarningBeepPwErrF = 0;

	  W_BEEP_Time1 = 0;
  }
  //2.手柄错误，未设置手柄
  else if (SysRunData.WarningBeepPwErrF == 1)  //响三声 100ms
  {
	  W_BEEP_Time = 0;

	  W_BEEP_Time1++;
	  switch (W_BEEP_Time1)
	  {
	    case 1 :
 	    case 21 :
	    case 41 :
		    Beep_OnOffCtrl(1);
	    break;
	    case 11 :
	    case 31 :
		    Beep_OnOffCtrl(0);
  	  break;
	    case 51 :
	    {
		    Beep_OnOffCtrl(0);
		    SysRunData.WarningBeepPwErrF = 0;
		    W_BEEP_Time1 = 0;
	    }
	    break;
	    default : break;
	  }
  }
  //3.提示
  else if (SysRunData.BeepTimeMS >= 10)
  {
	  W_BEEP_Time = 0;
	  W_BEEP_Time1 = 0;

	  switch (SysRunData.BeepTimeMS)
	  {
	    case 300 :
	    case 100 :
		    Beep_OnOffCtrl(1);
	    break;
	    case 200 :
	    case 0 :
		    Beep_OnOffCtrl(0);
	    break;
	    default : break;
	  }

	  SysRunData.BeepTimeMS -= 10;

	  if(SysRunData.BeepTimeMS == 0)
	    Beep_OnOffCtrl(0);
  }
  else
  {
	  W_BEEP_Time = 0;
	  W_BEEP_Time1 = 0;

	  Beep_OnOffCtrl(0);
  }
}







