//UI_ModelConfiguration.c

#include "UI_ModelConfiguration.h"
#include "lcd.h"
#include "data.h"
#include "flash.h"
#include "delay.h"
#include "pedal.h"
#include "iwdg.h"
#include "screenkey.h"


static uint8_t ZJ_Disp_HZ0[] = {14, 0xCE, 0xDE, 0xC9, 0xE8, 0xD6, 0xC3, 0xCA, 0xD6, 0xB1, 0xFA, 0x20}; //无设置手柄
static uint8_t ZJ_Disp_HZ1[] = {14, 0x44, 0x4C, 0x2D, 0x4A, 0x4B, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}; //DL-JK
static uint8_t ZJ_Disp_HZ2[] = {14, 0x44, 0x4C, 0x2D, 0x4A, 0x57, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}; //DL-JW
static uint8_t ZJ_Disp_HZ3[] = {14, 0x44, 0x4C, 0x2D, 0x4A, 0x5A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}; //DL-JZ
static uint8_t ZJ_Disp_HZ4[] = {14, 0x44, 0x4C, 0x2D, 0x4D, 0x41, 0x2F, 0x44, 0x4C, 0x2D, 0x4D, 0x44}; //DL-MA/DL-MD
static uint8_t ZJ_Disp_HZ5[] = {14, 0x44, 0x4C, 0x2D, 0x4D, 0x42, 0x2F, 0x44, 0x4C, 0x2D, 0x4D, 0x45}; //DL-MB/DL-ME
static uint8_t ZJ_Disp_HZ6[] = {14, 0x44, 0x4C, 0x2D, 0x4D, 0x43, 0x2F, 0x44, 0x4C, 0x2D, 0x4D, 0x46}; //DL-MC/DL-MF
static uint8_t ZJ_Disp_HZ7[] = {14, 0xCC, 0xD8, 0xB6, 0xA8, 0xD0, 0xCD, 0xBA, 0xC5, 0x20, 0x20, 0x20}; //特定型号

//============================================================================
// 函数名称: UI_ModelConfiguration_Fun()
// 功能描述: 1_机型配置
// 输　  入:
// 输    出: 0返回主界面 1下一页
// 函数说明:
//============================================================================
uint8_t UI_ModelConfiguration_Fun(void)
{
  uint8_t TimeCnt = 0;
  uint8_t temp = 0, model = SysModelConfig.type[0];

  while (1)
  {
    Delay_ms(2);

	  //扫描按键
	  //.................
	  ScreenKey_Scan();

	  if (++TimeCnt < 50)  //100ms
  	  continue ;

	  TimeCnt = 0;

	  Iwdg_Reset();  //喂狗

	  switch (SysRunData.KeyValue)
	  {
	    case KEY_JMB :
	    {
		    SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

		    temp = model & 0x01;
		    if(temp == 0x01)
		    {
		      model &= 0x0E;
		      LCD_Show_Picture(0x1201, Address_ICL_Page4_1);
		    }
		    else
		    {
		      model |= 0x01;
		      LCD_Show_Picture(0x1201, Address_ICL_Page4_2);
		    }
	    }
	    break;
	    case KEY_TMBA :
	    {
	      SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

		    temp = model & 0x02;
		    if(temp == 0x02)
		    {
		      model &= 0x0D;
		      LCD_Show_Picture(0x1202, Address_ICL_Page4_1);
		    }
		    else
		    {
		      model |= 0x02;
  		    LCD_Show_Picture(0x1202, Address_ICL_Page4_2);
		    }
	    }
	    break;
	    case KEY_TMBB :
	    {
		    SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

		    temp = model & 0x04;
		    if(temp == 0x04)
		    {
		      model &= 0x0B;
		      LCD_Show_Picture(0x1203, Address_ICL_Page4_1);
		    }
		    else
		    {
		      model |= 0x04;
		      LCD_Show_Picture(0x1203, Address_ICL_Page4_2);
		    }
	    }
	    break;
	    case KEY_TMBC :
	    {
		    SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

		    temp = model & 0x08;
		    if(temp == 0x08)
		    {
		      model &= 0x07;
		      LCD_Show_Picture(0x1204, Address_ICL_Page4_1);
		    }
		    else
		    {
		      model |= 0x08;
		      LCD_Show_Picture(0x1204, Address_ICL_Page4_2);
		    }
	    }
	    break;
	    case KEY_RETURN :
	    {
		    SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

		    return 0;
	    }
//	    break;
	    case KEY_NEXTPAGE :
	    {
	      SysRunData.BeepTimeMS = 100;

		    SysRunData.KeyValue = KEY_NONE;

//		    Pedal_ReadStorageHLValue();

		    return 1;
	    }
//		  break;
	    default : break;
	  }

	  //刷新UI
	  temp = model & 0x0F;
	  switch (temp)
	  {
	    case 0 :	 //0000
	    {
        LCD_HostModel_Update(0x4100, ZJ_Disp_HZ0);  // DL-JK
	    }
	    break;
	    case 1 :	 //0001     JMB
	    {
        LCD_HostModel_Update(0x4100, ZJ_Disp_HZ1);  // DL-JK
	    }
	    break;
	    case 2 :	 //0010     TMBA
	    {
		    LCD_HostModel_Update(0x4100, ZJ_Disp_HZ6);  // DL-MC  DL-MF
	    }
	    break;
	    case 4 :	 //0100    TMBB
	    {
		    LCD_HostModel_Update(0x4100, ZJ_Disp_HZ5);  // DL-MB  DL-ME
	    }
	    break;
	    case 5 :	 //0101     TMBB   JMB
	    {
		    LCD_HostModel_Update(0x4100, ZJ_Disp_HZ2);  // DL-JW
	    }
	    break;
	    case 7 :	 //0111     TMBB TMBA JMB
	    {
		    LCD_HostModel_Update(0x4100, ZJ_Disp_HZ3);  // DL-JZ
	    }
	    break;
	    case 14 :	 //1110   TMBC TMBB TMBA
	    {
		    LCD_HostModel_Update(0x4100, ZJ_Disp_HZ4);  // DL-MA  DL-MD
	    }
	    break;
	    case 3 :	 //0011   TMBA JMB
	    case 6 :	 //0110   TMBB TMBA
	    case 8 :	 //1000   TMBC
	    case 9 :	 //1001   TMBC JMB
	    case 10 :	 //1010   TMBC TMBA
	    case 11 :	 //1011   TMBC TMBA JMB
      case 12 :	 //1100   TMBC TMBB
	    case 13 :	 //1101   TMBC TMBB JMB
	    case 15 :	 //1111   TMBC TMBB TMBA JMB
	    {
	      LCD_HostModel_Update(0x4100, ZJ_Disp_HZ7);  // 特定型号
	    }
	    break;
	    default : break;
    }
  }
}








