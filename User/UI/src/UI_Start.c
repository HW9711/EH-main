//UI_Start.c

#include "UI_Start.h"
#include "UI_FootPedalCalibration.h"
#include "iwdg.h"
#include "screenkey.h"
#include "sscBEEP.h"
#include "delay.h"
#include "lcd.h"
#include "screen_address.h"

#include <stdint.h>

#define UI_START_CALIBRATION_PRESS_COUNT 5U /* 启动页必须连续接收五次定标按钮事件才允许进入Page3。 */

//============================================================================
// 函数名称: UI_Start_Fun()
// 功能描述: 起始页
// 输　  入:
// 输    出: 0返回主界面 1下一页
// 函数说明: 开机扫描”LOGO点击动作“ 进入厂家配置 4s等待...
//============================================================================
/*
 * 函数功能：保持 EX8 启动页等待，并在正常业务任务创建前接收脚踏定标入口。
 * 输入参数：无。
 * 返回参数：无。
 */
void UI_Start_Fun(void)
{
  uint8_t TimeCnt = 0U; /* 每 20 次 2ms 循环喂狗一次，启动等待期间保持看门狗在线。 */
  uint8_t key_value = KEY_NONE; /* 启动页一次性事件读取后立即清空，避免同一次触摸重复进入。 */
  uint8_t calibration_press_count = 0U; /* 只在本次约4秒启动窗口内累计定标按钮点击次数。 */
  uint16_t DelayTime = 2000U; /* 2000 次乘 2ms 形成约 4 秒启动页入口窗口。 */

  while (DelayTime--)
  {
    Delay_ms(2); /* 保持原启动页约 4 秒总等待节奏。 */
    ScreenKey_Scan(); /* 正常屏幕任务尚未创建，由启动流程独占读取 UART6。 */

    key_value = ScreenKey_LegacyEventTake(); /* 每轮消费一次启动页事件，非入口事件保持静默。 */
    if (key_value == KEY_CONTINUOUSCLICK) /* DWIN每次独立触摸返回一个key1事件，前四次只累计不切页。 */
    {
      if (calibration_press_count < UI_START_CALIBRATION_PRESS_COUNT)
      {
        calibration_press_count++; /* 计数保持饱和，避免异常重复帧造成8位计数回绕。 */
      }

      if (calibration_press_count >= UI_START_CALIBRATION_PRESS_COUNT)
      {
        calibration_press_count = 0U; /* 第五次确认后清零本地计数，再进入不会主动返回的定标循环。 */
        LCD_ForceShow_Which_Map(UIDP_LCD_PAGE_PEDAL_CALIBRATION); /* 达到五次门槛后才显示Page3，避免启动页单次误触。 */
        Beep_Pulse100ms(); /* 蜂鸣任务尚未创建，直接输出100ms按键音确认定标入口已经生效。 */
        Delay_ms(5U); /* 给DWIN背景页切换留出发送间隔，再开始刷新定标数据。 */
        UI_FootPedalCalibration_Fun(); /* 进入独占定标循环；电机、泵、外控和正常脚踏任务均尚未创建。 */
      }
    }

    if (++TimeCnt < 20U)
    {
      continue; /* 未到约 40ms 喂狗间隔时继续等待，避免每 2ms 都访问看门狗。 */
    }

    TimeCnt = 0U; /* 开始下一轮约 40ms 喂狗计数。 */
    Iwdg_Reset(); /* 启动页或等待入口期间持续喂狗，防止被误判为主程序卡死。 */
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





