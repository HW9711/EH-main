//UI_Start.c

#include "UI_Start.h"
#include "UI_FootPedalCalibration.h"
#include "iwdg.h"
#include "delay.h"
#include "data.h"
#include "screenkey.h"
#include "lcd.h"
#include "sscBEEP.h"
#include "screen_address.h"

#include <stdint.h>

//============================================================================
// 函数名称: UI_Start_Fun()
// 功能描述: 起始页
// 输　  入:
// 输    出: 0返回主界面 1下一页
// 函数说明: 开机扫描”LOGO点击动作“ 进入厂家配置 4s等待...
//============================================================================
void UI_Start_Fun(void)
{
  uint8_t TimeCnt = 0;
  uint16_t DelayTime = 2000;

  while (DelayTime--)
  {
    Delay_ms(2);

    //按键扫描
	  //....................
    ScreenKey_Scan();

	  if(++TimeCnt < 20)
	    continue ;

	  TimeCnt = 0;

	  Iwdg_Reset();

	  if (ScreenKey_LegacyEventTake() == KEY_CONTINUOUSCLICK)
	  {
	    // 启动页按键提示统一进入新蜂鸣队列，不再写旧蜂鸣时长状态。
	    SendKeyBeepMessage(1U);

			LCD_Show_Which_Map(3);
			LCD_Show_Which_Map(0);
			LCD_Show_Which_Map(3);

      Delay_ms(5);

			UI_FootPedalCalibration_Fun();
			
		}
  }
}



//============================================================================
// 函数名称: UI_Show_init()
// 功能描述: UI主界面初始化
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void UI_Show_init(void)
{
  LCD_Show_Which_Map(UIDP_LCD_PAGE_MAIN_RUN);        //新屏开机后主运行页固定为 page4，保持老成功版启动页和运行页分离
  LCD_Disappear_Picture(UIDP_LCD_VP_ALARM_TIP);//清掉新屏报警提示区，后续完整区域刷新由 UIDP 任务统一接管

}





