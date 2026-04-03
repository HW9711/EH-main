//UI_Password.c

#include "UI_Password.h"

#include "iwdg.h"
#include "delay.h"
#include "data.h"
#include "lcd.h"
#include "screen.h"
#include "common.h"
#include "screenkey.h"

static uint8_t Password[8] = { 0 }; //1 2 2 3 3 1
static uint8_t NextStep = 0;

//============================================================================
//返回 1返回 0正常继续
//============================================================================
uint8_t UI_Password_KeyValueData(void)
{
  Screen_Password_Input(NextStep);

  if (NextStep == 0)
    Common_Memset(0, Password, 8);

  switch (SysRunData.KeyValue)
  {
	  case KEY_ZERO :
	  case KEY_ONE :
	  case KEY_TWO :
	  case KEY_THREE :
	  case KEY_FOUR :
	  case KEY_FIVE :
	  case KEY_SIX :
	  case KEY_SEVEN :
	  case KEY_EIGHT :
	  case KEY_NINE :
	  {
	    Password[NextStep] = SysRunData.KeyValue;
	    SysRunData.KeyValue = KEY_NONE;
	    SysRunData.BeepTimeMS = 100;

	    NextStep += 1;
    }
	  break;
	  case KEY_REINPUT :
	  {
	    SysRunData.KeyValue = KEY_NONE;
	    SysRunData.BeepTimeMS = 100;

	    NextStep = 0;
	  }
	  break;
	  case KEY_RETURN :
	  {
	    SysRunData.KeyValue = KEY_NONE;
	    SysRunData.BeepTimeMS = 100;

	    return 1;
	  }
//	  break;
	  default : break;
  }

  return 0;
}

//============================================================================
// 函数名称: UI_Password_Input()
// 功能描述: 1_输入密码界面
// 输　  入:
// 输    出: 0返回主界面 1密码正确
// 函数说明: 密码122331
//============================================================================
uint8_t UI_Password_Input(void)   //Password_Input
{
  uint8_t TimeCnt = 0;

  NextStep = 0;

  while(1)
  {
    Delay_ms(2);

	  //扫描按键
    //...................
    ScreenKey_Scan();

    if (++TimeCnt < 50)
      continue ;

    TimeCnt = 0;

	  Iwdg_Reset();  //喂狗

	  switch(NextStep)
	  {
	    case 0 :
	    case 1 :
	    case 2 :
	    case 3 :
	    case 4 :
      case 5 :
	    {
		    if (UI_Password_KeyValueData())
		      return 0;
	    }
	    break;
	    case 6:
	    {
	      Screen_Password_Input(6);

		    if(SysRunData.KeyValue == KEY_RETURN) //返回主界面
		    {
		      SysRunData.KeyValue = KEY_NONE;
		      SysRunData.BeepTimeMS = 100;

		      return 0;
		    }
		    else if ((Password[0] == 1) && (Password[1] == 2) && (Password[2] == 2) &&
				         (Password[3] == 3) && (Password[4] == 3) && (Password[5] == 1))
		    {
		      SysRunData.BeepTimeMS = 100;

		      return 1;
		    }
		    else
		    {
		      SysRunData.WarningBeepPwErrF = 1;
		      NextStep += 1;
		    }
	    }
	    break;
	    case 7:
	    {
	      Screen_Password_Input(7);

		    Delay_ms(1200);

		    NextStep = 0;
	    }
	    break;
	    default : break;
	  }
  }
}








