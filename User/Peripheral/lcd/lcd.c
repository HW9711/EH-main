//lcd.c

#include "lcd.h"
#include "uart6.h"
#include "common.h"
#include "delay.h"

typedef struct
{
  uint8_t UIDisplay0x1403;  // 往复角度图片显示状态缓存，0=隐藏，1=显示。
  uint8_t UIDisplay0x1303;  // 转速栏图片状态缓存，避免重复刷新同一状态。
  uint8_t UIDisplay0x1304;  // 频率/挡位栏图片状态缓存，供 LCD 局部刷新判断。
  uint8_t UIDisplay0x1305;  // B 泵区域图片状态缓存。
  uint8_t UIDisplay0x1311;  // 往复方向图片状态缓存。
  uint8_t UIDisplay0x1310;  // 正向方向图片状态缓存。
  uint8_t UIDisplay0x1312;  // 脚踏控制图片状态缓存。
  uint8_t UIDisplay0x1313;  // 手控图片状态缓存。
  uint8_t UIDisplay0x1500;  // A 手柄图片状态缓存。
  uint8_t UIDisplay0x1501;  // B 手柄图片状态缓存。
  uint8_t UIDisplay0x1318;  // A 泵区域图片状态缓存。
} LCD_DisplayCache_t;

static uint8_t s_lcd_background_page = 0;                 // 当前背景页缓存，只服务 LCD 背景页去重发送。
static volatile LCD_DisplayCache_t s_lcd_display_cache = { 0 };  // LCD 图片显示状态缓存，写入具备调试可见性，不再依赖旧 UI 全局状态。

//============================================================================
// 函数名称: LCD_Show_Which_Map()
// 功能描述: 显示某一张背景图片
// 输　  入: MapAddr：背景地址号
// 输    出: 无
// 函数说明: 同一个背景页最多发两次
//============================================================================
void LCD_Show_Which_Map(uint8_t MapAddr)
{
	//5A A5 07 82 0084 5A01 XXXX
  uint8_t dat[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01};

  static uint8_t SendCnt = 1;

  if (MapAddr == s_lcd_background_page)
  {
    if (SendCnt > 0)
	    SendCnt--;
	  else
	    return ;
  }
  else
	  SendCnt = 1;

  s_lcd_background_page = MapAddr;

  dat[8] = (MapAddr >> 8) & 0x00ff;
  dat[9] = MapAddr & 0x00ff;

  Uart6_SendPacket(dat, 10);
}

//============================================================================
// 函数名称: LCD_Disappear_Number()
// 功能描述: 隐藏某个数据
// 输　  入: DataAddr：数据显示地址
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Disappear_Number(uint16_t DataAddr)
{
  //5A A5 07 82 5420 XXXX XXXX
  uint8_t dat[10] = {0x5A, 0xA5, 0x05, 0x82};

  dat[4] = (DataAddr >> 8) & 0x00ff;
  dat[5] = DataAddr & 0x00ff;

  dat[6] = 0xFF;
  dat[7] = 0x00;

  Uart6_SendPacket(dat, 8);
}

//============================================================================
// 函数名称: LCD_Disappear_Picture()
// 功能描述: 隐藏某个图片
// 输　  入: PicAddr：图片显示地址
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Disappear_Picture(uint16_t PicAddr)
{
  uint8_t dat[8] = {0x5A, 0xA5, 0x05, 0x82};

  dat[4] = (PicAddr >> 8) & 0x00ff;
  dat[5] =  PicAddr & 0x00ff;

  dat[6] = 0xff;
  dat[7] = 0xff;

  Uart6_SendPacket(dat, 8);

  switch (PicAddr)
  {
	  case 0x1606 : s_lcd_display_cache.UIDisplay0x1403 = 0;	break;  // 往复角度图片隐藏。
	  case 0x1600 : s_lcd_display_cache.UIDisplay0x1303 = 0; break;  // 转速栏隐藏。
	  case 0x1601 : s_lcd_display_cache.UIDisplay0x1304 = 0; break;  // 频率或挡位栏隐藏。
	  case 0x1506 : s_lcd_display_cache.UIDisplay0x1305 = 0; break;  // B 泵区域隐藏。
	  case 0x1502 : s_lcd_display_cache.UIDisplay0x1318 = 0; break;  // A 泵区域隐藏。

		case 0x1602 :
    {
			s_lcd_display_cache.UIDisplay0x1311 = 0;
		  s_lcd_display_cache.UIDisplay0x1310 = 0;
			break;
		}		
//	  case 0x1311 : s_lcd_display_cache.UIDisplay0x1311 = 0; break;  //往复
//	  case 0x1310 : s_lcd_display_cache.UIDisplay0x1310 = 0; break;  //正向

	  case 0x1312 : s_lcd_display_cache.UIDisplay0x1312 = 0; break;  // 脚踏控制图标隐藏。
	  case 0x1313 : s_lcd_display_cache.UIDisplay0x1313 = 0; break;  // 手控图标隐藏。
	  default : break;
  }
}

//============================================================================
// 函数名称: LCD_Show_Number()
// 功能描述: 显示某个数据
// 输　  入: DatdAddr：数据显示地址
//           Data：显示的数据
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Show_Number(uint16_t DatdAddr, uint16_t Data)
{
  //5A A5 07 82 5420 XXXX XXXX
  uint8_t dat[10]={0x5A, 0xA5, 0x05, 0x82};

  dat[4] = (DatdAddr >> 8) & 0x00ff;
  dat[5] = DatdAddr & 0x00ff;

  dat[6] = (Data >> 8) & 0x00ff;
  dat[7] = Data & 0x00ff;

  Uart6_SendPacket(dat, 8);
}

//============================================================================
// 函数名称: LCD_Show_Picture()
// 功能描述: 显示某个图片
// 输　  入: PicAddr：图片的显示地址
//           PicNum：图片的编号
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Show_Picture(uint16_t PicAddr, uint16_t PicNum)
{
  uint8_t dat[8]={0x5A, 0xA5, 0x05, 0x82};

  dat[4] = (PicAddr >> 8) & 0x00ff;
  dat[5] = PicAddr & 0x00ff;

  dat[6] = (PicNum >> 8) & 0x00ff;
  dat[7] = PicNum & 0x00ff;

  Uart6_SendPacket(dat, 8);

  switch (PicAddr)
  {
	  case 0x1606 :
	  {
	    if (PicNum == 400)  //往复角度图片显
		    s_lcd_display_cache.UIDisplay0x1403 = 1;
	  }
	  break;
	  case 0x1600 :
	  {
	    if (PicNum == 340)  //转速 灰
		    s_lcd_display_cache.UIDisplay0x1303 = 1;
	    else if (PicNum == 341)
		    s_lcd_display_cache.UIDisplay0x1303 = 2;
	  }
	  break;
	  case 0x1601 :
	  {
	    if ((PicNum == 350) || (PicNum == 355))  //灰色
		    s_lcd_display_cache.UIDisplay0x1304 = 1;
	    else if (PicNum == 351)  //频率栏高亮
		    s_lcd_display_cache.UIDisplay0x1304 = 2;
	    else if (PicNum == 352)  //I档选中
		    s_lcd_display_cache.UIDisplay0x1304 = 3;
	    else if (PicNum == 353)  //Ⅱ档选中
		    s_lcd_display_cache.UIDisplay0x1304 = 4;
	    else if (PicNum == 354)  //Ⅲ档选中
		    s_lcd_display_cache.UIDisplay0x1304 = 5;
	  }
	  break;
		case 0x1500 :
		{
	    if (PicNum == 106)  //A手柄未连接		
			{
				s_lcd_display_cache.UIDisplay0x1500 = 0;
			}
			else
      {
				s_lcd_display_cache.UIDisplay0x1500 = 1;
			}
		}	  
		break;			
		case 0x1501 :			
		{
	    if (PicNum == 206)  //B手柄未连接	
			{
				s_lcd_display_cache.UIDisplay0x1501 = 0;
			}
			else
      {
				s_lcd_display_cache.UIDisplay0x1501 = 1;
			}				
		}	  
		break;
	  case 0x1506 :
	  {
	    if (PicNum == 302)  //流量 灰
		    s_lcd_display_cache.UIDisplay0x1305 = 1;
	    else //if (PicNum == 227)
		    s_lcd_display_cache.UIDisplay0x1305 = 2;
	  }
	  break;	
 		
	  case 0x1502 :
	  {
	    if (PicNum == 302)  //流量 灰
		    s_lcd_display_cache.UIDisplay0x1318 = 1;
	    else //if (PicNum == 227)
		    s_lcd_display_cache.UIDisplay0x1318 = 2;
	  }
	  break;		
	  case 0x1312 :
	  {
	    if (PicNum == 260)  //未选中
		    s_lcd_display_cache.UIDisplay0x1312 = 2;
	    else if (PicNum == 261)  //选中
		    s_lcd_display_cache.UIDisplay0x1312 = 3;
	    else
		    s_lcd_display_cache.UIDisplay0x1312 = 1;
	  }
	  break;
	  case 0x1313 :
	  {
	    if (PicNum == 262)  //未选中
	      s_lcd_display_cache.UIDisplay0x1313 = 2;
	    else if (PicNum == 263)  //选中
		    s_lcd_display_cache.UIDisplay0x1313 = 3;
	    else
		    s_lcd_display_cache.UIDisplay0x1313 = 1;
	  }
	  break;
	  case 0x1602 :
	  {		
	    if (PicNum == 360)  //未选中
			{
		    s_lcd_display_cache.UIDisplay0x1310 = 0;
			  s_lcd_display_cache.UIDisplay0x1311 = 0;
			}
	    else if (PicNum == 361)  //正3
			{	
		    s_lcd_display_cache.UIDisplay0x1310 = 1;
			  s_lcd_display_cache.UIDisplay0x1311 = 1;
			}
	    else if (PicNum == 362)  //往复3
			{	
				s_lcd_display_cache.UIDisplay0x1310 = 2;
			  s_lcd_display_cache.UIDisplay0x1311 = 1;
			}
			else if (PicNum == 363)  //反3
			{	
				s_lcd_display_cache.UIDisplay0x1310 = 3;
			  s_lcd_display_cache.UIDisplay0x1311 = 1;
			}
			else if (PicNum == 364)  //正2
			{	
				s_lcd_display_cache.UIDisplay0x1310 = 1;
			  s_lcd_display_cache.UIDisplay0x1311 = 0;
			}
			else if (PicNum == 365)  //反2
			{	
				s_lcd_display_cache.UIDisplay0x1310 = 3;
			  s_lcd_display_cache.UIDisplay0x1311 = 0;
			}
			else    //未选中
			{
		    s_lcd_display_cache.UIDisplay0x1310 = 0;
			  s_lcd_display_cache.UIDisplay0x1311 = 0;
			}	
	  }
	  break;		
//	  case 0x1311 :
//	  {
//	    if (PicNum == 256)  //未选中
//		    s_lcd_display_cache.UIDisplay0x1311 = 2;
//	    else if (PicNum == 257)  //选中
//		    s_lcd_display_cache.UIDisplay0x1311 = 3;
//	    else
//		    s_lcd_display_cache.UIDisplay0x1311 = 1;
//	  }
//	  break;
//	  case 0x1310 :
//	  {
//	    if (PicNum == 250)  //未选中
//		    s_lcd_display_cache.UIDisplay0x1310 = 2;
//	    else if (PicNum == 251)  //选中
//		    s_lcd_display_cache.UIDisplay0x1310 = 3;
//	    else if (PicNum == 254)  //未选中 长
//		    s_lcd_display_cache.UIDisplay0x1310 = 4;
//	    else if (PicNum == 255)  //选中 长
//		    s_lcd_display_cache.UIDisplay0x1310 = 5;
//      else
//		    s_lcd_display_cache.UIDisplay0x1310 = 1;
//	  }
//	  break;
//	  case 0x1316 :
//	  {
//	    if (PicNum == 303)  //未选中
//		    旧反向图标缓存分支已停用，当前方向图标由 0x1602 组合图缓存维护。
//	    else if (PicNum == 304)  //选中
//		    旧反向图标缓存分支已停用，当前方向图标由 0x1602 组合图缓存维护。
//	    else
//		    旧反向图标缓存分支已停用，当前方向图标由 0x1602 组合图缓存维护。
//	  }
//	  break;
	  default : break;
  }
}

//============================================================================
// 函数名称: LCD_Show_2byte_Number()
// 功能描述: 2字节数据更新
// 输　  入: Addr：数据的显示地址
//           Data：更新的数据
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Show_2byte_Number(uint16_t Addr, uint16_t Data)
{
  //5A A5 05 82 5420 XXXX
  uint8_t dat[8] ={0x5A, 0xA5, 0x05, 0x82};

  dat[4] = (Addr >> 8) & 0x00ff;
  dat[5] = Addr & 0x00ff;

  dat[6] = (Data >> 8) & 0x00ff;
  dat[7] = Data & 0x00ff;

  Uart6_SendPacket(dat, 8);
}

//============================================================================
// 函数名称: LCD_Show_4byte_Number()
// 功能描述: 4字节数据更新
// 输　  入: Addr：数据的显示地址
//           Data：更新的数据
// 输    出: 无
// 函数说明:
//============================================================================
void LCD_Show_4byte_Number(uint16_t Addr, uint32_t data)
{
  //5A A5 07 82 5420 XXXX XXXX
  uint8_t dat[10]={0x5A, 0xA5, 0x07, 0x82};

  dat[4] = (Addr >> 8) & 0x00ff;
  dat[5] = Addr & 0x00ff;

  dat[6] = (data >> 24) & 0x000000ff;
  dat[7] = (data >> 16) & 0x000000ff;
  dat[8] = (data >> 8) & 0x000000ff;
  dat[9] = data & 0x000000ff;

  Uart6_SendPacket(dat, 10);
}

//============================================================================
// 函数名称: LCD_IntegratedCutterData_Update()
// 功能描述: 一体式(3号接口)刀具参数更新
// 输　  入: Addr：数据参数的显示地址
//           Length：长度
//           Diameter：直径
//           Angle：角度
// 输    出: 无
// 函数说明: 4_运行界面、5_运行界面，一体式刀具参数（文字显示），样式：
//  规     格    :      φ     4  .  0   、   1  1  0   m   m   、          0    °
// B9E6   B8F1  3A 20  A6D5  34 2E 30 A1A2  31 31 30  6D  6D  A1A2  20    30   A1E3
//============================================================================
void LCD_IntegratedCutterData_Update(uint16_t Addr, uint16_t Length, uint8_t Diameter, uint8_t Angle)
{
  uint8_t LengthTemp[3] = { 0 }, DiameterTemp[2] = { 0 }, AngleTemp[3] = { 0 };

  uint8_t dat[64] = {0x5A, 0xA5, 0x1E, 0x82};

  static uint16_t LengthLast = 0xff;
  static uint8_t DiameterLast = 0xff, AngleLast = 0xff;

  if ((Length == LengthLast) && (Diameter == DiameterLast) && (Angle == AngleLast))
    return ;

  LengthLast = Length;
  DiameterLast = Diameter;
  AngleLast = Angle;

  dat[4] = (Addr >> 8) & 0x00ff ;
  dat[5] = Addr & 0x00ff ;

  LengthTemp[2] = Length / 100;
  LengthTemp[1] = Length / 10 % 10;
  LengthTemp[0] = Length % 10;

  DiameterTemp[1] = Diameter / 10 % 10;
  DiameterTemp[0] = Diameter % 10;

  AngleTemp[2] = Angle / 100;
  AngleTemp[1] = Angle / 10 % 10;
  AngleTemp[0] = Angle % 10;

  dat[2] = 31; //数据长度

  if ((Length == 0) && (Diameter == 0) && (Angle == 0))
  {
	  Common_Memset(0x20, &dat[6], 28);
  }
  else
  {
		#if 1
	  dat[6] = 0xB9;
	  dat[7] = 0xE6;  //规

	  dat[8] = 0xB8;
	  dat[9] = 0xF1;  //格
		#else
		 dat[6] = 'S';
	  dat[7] = 'p';  //规

	  dat[8] = 'e';
	  dat[9] = 'c';  //格
		#endif

	  dat[10] = 0x3A; //:
	  dat[11] = 0x20; //空格

	  dat[12] = 0xA6;
	  dat[13] = 0xD5; //φ

	  dat[14] = DiameterTemp[1] + 0x30; //直径十位
	  dat[15] = 0x2E;  //.
	  dat[16] = DiameterTemp[0] + 0x30; //直径个位
	  dat[17] = 0xA1;
	  dat[18] = 0xA2;  //、
	  dat[19] = 0x20;  //空格

	  dat[20] = LengthTemp[2] + 0x30;
	  dat[21] = LengthTemp[1] + 0x30;
	  dat[22] = LengthTemp[0] + 0x30;
	  dat[23] = 0x6D;  //'m'
	  dat[24] = 0x6D;  //'m'
	  dat[25] = 0xA1;
	  dat[26] = 0xA2;  //、
	  dat[27] = 0x20;

	  if(AngleTemp[2] > 0)//百位不为0
	  {
   	  dat[28] = AngleTemp[2] + 0x30;
	    dat[29] = AngleTemp[1] + 0x30;
	    dat[30] = AngleTemp[0] + 0x30;
	    dat[31] = 0xA1;
	    dat[32] = 0xE3;  //°
	    dat[33] = 0x20;
	  }
	  else if(AngleTemp[1] > 0)//百位为0，十位不为0
	  {
	    dat[28] = AngleTemp[1] + 0x30;
	    dat[29] = AngleTemp[0] + 0x30 ;
	    dat[30] = 0xA1;
	    dat[31] = 0xE3;  //°
	    dat[32] = 0x20;
	    dat[33] = 0x20;
	  }
	  else //百位为0，十位为0
	  {
	    dat[28] = AngleTemp[0] + 0x30 ;
	    dat[29] = 0xA1;
	    dat[30] = 0xE3;  //°
	    dat[31] = 0x20;
	    dat[32] = 0x20;
	    dat[33] = 0x20;
	  }
  }

  Uart6_SendPacket(dat, 34);
//	Delay_ms(100);
}

//============================================================================
// 函数名称: LCD_HostModel_Update()
// 功能描述: 一体式(3号接口)刀具参数更新
// 输　  入: Addr：字符显示地址
//           *Str：字符串
// 输    出: 无
// 函数说明: 2_机型选择界面，主机型号（文字显示）
//============================================================================
void LCD_HostModel_Update(uint32_t Addr, uint8_t *Str)
{
  uint8_t dat[10] = {0x5A, 0xA5, 0, 0x82};

  dat[2] = Str[0];  //数据长度

  dat[4] = (Addr >> 8) & 0x00ff ;
  dat[5] = Addr & 0x00ff ;

  Uart6_SendPacket(dat, 6);
  Uart6_SendPacket(Str + 1, Str[0] - 3);
}


