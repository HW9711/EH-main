//UI_Start.c

#include "UI_Start.h"
#include "iwdg.h"
#include "delay.h"
#include "lcd.h"
#include "screen_address.h"

#include <stdint.h>

//============================================================================
// 函数名称: UI_Start_Fun()
// 功能描述: 起始页
// 输　  入:
// 输    出: 0返回主界面 1下一页
// 函数说明: 开机扫描”LOGO点击动作“ 进入厂家配置 4s等待...
//============================================================================
/*
 * 函数功能：保持 EX8 启动页上电等待节奏，并周期性喂狗；旧屏 LOGO 连击入口已停用。
 * 输入参数：无。
 * 返回参数：无。
 */
void UI_Start_Fun(void)
{
  uint8_t TimeCnt = 0;
  uint16_t DelayTime = 2000;

  while (DelayTime--)
  {
    Delay_ms(2);

	  if(++TimeCnt < 20)
	    continue ;

	  TimeCnt = 0;

	  Iwdg_Reset();
	  /* EX8 启动页不再保留旧 LOGO 连击入口；这里只保留启动页等待和喂狗节奏，标定页由新屏专用入口维护。 */
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





