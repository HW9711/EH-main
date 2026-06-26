//lcd.c

#include "lcd.h"
#include "screen_address.h"
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

/*
 * 函数功能：强制刷新指定背景页，绕过普通 LCD_Show_Which_Map() 的同页去重。
 * 输入参数：MapAddr 为 DWIN 背景页号，主运行页固定传 0。
 * 返回参数：无。
 */
void LCD_ForceShow_Which_Map(uint8_t MapAddr)
{
  uint8_t dat[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01}; /* DWIN 背景页切换命令，开机初始化必须完整重发。 */

  s_lcd_background_page = MapAddr; /* 强制切页成功发送前同步软件缓存，后续普通切页仍按当前页去重。 */
  dat[8] = (MapAddr >> 8) & 0x00ff; /* 背景页号高字节，当前工程实际只使用低 8 位页号。 */
  dat[9] = MapAddr & 0x00ff;        /* 背景页号低字节，page0 主运行页会写入 0。 */

  Uart6_SendPacket(dat, 10); /* 直接走屏幕串口发送，不受同页 SendCnt 影响，避免启动页底图残留。 */
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
	  case UIDP_LCD_LEGACY_VP_OSC_ANGLE_CACHE : s_lcd_display_cache.UIDisplay0x1403 = 0;	break;  // 往复角度图片隐藏。
	  case UIDP_LCD_LEGACY_VP_SPEED_AREA_CACHE : s_lcd_display_cache.UIDisplay0x1303 = 0; break;  // 转速栏隐藏。
	  case UIDP_LCD_LEGACY_VP_FREQ_GEAR_CACHE : s_lcd_display_cache.UIDisplay0x1304 = 0; break;  // 频率或挡位栏隐藏。
	  case UIDP_LCD_LEGACY_VP_PUMP_B_CACHE : s_lcd_display_cache.UIDisplay0x1305 = 0; break;  // B 泵区域隐藏。
	  case UIDP_LCD_LEGACY_VP_PUMP_A_CACHE : s_lcd_display_cache.UIDisplay0x1318 = 0; break;  // A 泵区域隐藏。

		case UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE :
    {
			s_lcd_display_cache.UIDisplay0x1311 = 0;
		  s_lcd_display_cache.UIDisplay0x1310 = 0;
			break;
		}
//	  case UIDP_LCD_LEGACY_VP_DIR_OSC_CACHE : s_lcd_display_cache.UIDisplay0x1311 = 0; break;  //往复
//	  case UIDP_LCD_LEGACY_VP_DIR_FORWARD_CACHE : s_lcd_display_cache.UIDisplay0x1310 = 0; break;  //正向

	  case UIDP_LCD_LEGACY_VP_CONTROL_FOOT_CACHE : s_lcd_display_cache.UIDisplay0x1312 = 0; break;  // 脚踏控制图标隐藏。
	  case UIDP_LCD_LEGACY_VP_CONTROL_HANDLE_CACHE : s_lcd_display_cache.UIDisplay0x1313 = 0; break;  // 手控图标隐藏。
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
	  case UIDP_LCD_LEGACY_VP_OSC_ANGLE_CACHE :
	  {
	    if (PicNum == 400)  //往复角度图片显
		    s_lcd_display_cache.UIDisplay0x1403 = 1;
	  }
	  break;
	  case UIDP_LCD_LEGACY_VP_SPEED_AREA_CACHE :
	  {
	    if (PicNum == 340)  //转速 灰
		    s_lcd_display_cache.UIDisplay0x1303 = 1;
	    else if (PicNum == 341)
		    s_lcd_display_cache.UIDisplay0x1303 = 2;
	  }
	  break;
	  case UIDP_LCD_LEGACY_VP_FREQ_GEAR_CACHE :
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
		case UIDP_LCD_LEGACY_VP_HANDLE_A_CACHE :
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
		case UIDP_LCD_LEGACY_VP_HANDLE_B_CACHE :
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
	  case UIDP_LCD_LEGACY_VP_PUMP_B_CACHE :
	  {
	    if (PicNum == 302)  //流量 灰
		    s_lcd_display_cache.UIDisplay0x1305 = 1;
	    else //if (PicNum == 227)
		    s_lcd_display_cache.UIDisplay0x1305 = 2;
	  }
	  break;

	  case UIDP_LCD_LEGACY_VP_PUMP_A_CACHE :
	  {
	    if (PicNum == 302)  //流量 灰
		    s_lcd_display_cache.UIDisplay0x1318 = 1;
	    else //if (PicNum == 227)
		    s_lcd_display_cache.UIDisplay0x1318 = 2;
	  }
	  break;
	  case UIDP_LCD_LEGACY_VP_CONTROL_FOOT_CACHE :
	  {
	    if (PicNum == 260)  //未选中
		    s_lcd_display_cache.UIDisplay0x1312 = 2;
	    else if (PicNum == 261)  //选中
		    s_lcd_display_cache.UIDisplay0x1312 = 3;
	    else
		    s_lcd_display_cache.UIDisplay0x1312 = 1;
	  }
	  break;
	  case UIDP_LCD_LEGACY_VP_CONTROL_HANDLE_CACHE :
	  {
	    if (PicNum == 262)  //未选中
	      s_lcd_display_cache.UIDisplay0x1313 = 2;
	    else if (PicNum == 263)  //选中
		    s_lcd_display_cache.UIDisplay0x1313 = 3;
	    else
		    s_lcd_display_cache.UIDisplay0x1313 = 1;
	  }
	  break;
	  case UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE :
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
//	  case UIDP_LCD_LEGACY_VP_DIR_OSC_CACHE :
//	  {
//	    if (PicNum == 256)  //未选中
//		    s_lcd_display_cache.UIDisplay0x1311 = 2;
//	    else if (PicNum == 257)  //选中
//		    s_lcd_display_cache.UIDisplay0x1311 = 3;
//	    else
//		    s_lcd_display_cache.UIDisplay0x1311 = 1;
//	  }
//	  break;
//	  case UIDP_LCD_LEGACY_VP_DIR_FORWARD_CACHE :
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
//	  case UIDP_LCD_LEGACY_VP_DIR_REVERSE_CACHE :
//	  {
//	    if (PicNum == 303)  //未选中
//		    旧反向图标缓存分支已停用，当前方向图标由 UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE 组合图缓存维护。
//	    else if (PicNum == 304)  //选中
//		    旧反向图标缓存分支已停用，当前方向图标由 UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE 组合图缓存维护。
//	    else
//		    旧反向图标缓存分支已停用，当前方向图标由 UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE 组合图缓存维护。
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

static uint8_t s_lcd_integrated_cutter_raw_integer_mode = 0U; /* 公共接头 EPC 本次下发的原始整数显示开关，只在 lcd.c 内部临时使用。 */

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
  uint8_t LengthTemp[3] = { 0 }, DiameterTemp[3] = { 0 }, AngleTemp[3] = { 0 };

  uint8_t dat[64] = {0x5A, 0xA5, 0x1E, 0x82};

  static uint16_t LengthLast = 0xff;
  static uint8_t DiameterLast = 0xff, AngleLast = 0xff;
  static uint8_t ModeLast = 0xff;
  uint8_t raw_integer_mode = s_lcd_integrated_cutter_raw_integer_mode; /* 记录本次下发是否为公共接头 EPC 原始整数格式，避免影响旧小数直径路径。 */
  uint8_t diameter_decimal_extra = 0U; /* 直径小数格式达到 10.0mm 以上时多占 1 个字符，长度和角度文本需要同步右移。 */

  if ((Length == LengthLast) && (Diameter == DiameterLast) && (Angle == AngleLast) && (raw_integer_mode == ModeLast))
    return ;

  LengthLast = Length;
  DiameterLast = Diameter;
  AngleLast = Angle;
  ModeLast = raw_integer_mode;

  dat[4] = (Addr >> 8) & 0x00ff ;
  dat[5] = Addr & 0x00ff ;

  LengthTemp[2] = Length / 100;
  LengthTemp[1] = Length / 10 % 10;
  LengthTemp[0] = Length % 10;

  DiameterTemp[2] = Diameter / 100;
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

	  if(raw_integer_mode != 0U)
	  {
		  if(DiameterTemp[2] > 0U)
		  {
			  dat[14] = DiameterTemp[2] + 0x30; //公共接头 EPC 直径百位不为 0 时占满三位整数显示。
			  dat[15] = DiameterTemp[1] + 0x30; //公共接头 EPC 直径十位跟随百位显示，保留原始整数。
			  dat[16] = DiameterTemp[0] + 0x30; //公共接头 EPC 直径个位跟随百位显示，支持 100~255 范围。
		  }
		  else if(DiameterTemp[1] > 0U)
		  {
			  dat[14] = DiameterTemp[1] + 0x30; //公共接头 EPC 直径十位，0x10 解析后的 16 显示为字符 '1'。
			  dat[15] = DiameterTemp[0] + 0x30; //公共接头 EPC 直径个位，0x10 解析后的 16 显示为字符 '6'。
			  dat[16] = 0x20; //公共接头 EPC 两位直径不显示小数点，第三个字符用空格擦掉旧小数位。
		  }
		  else
		  {
			  dat[14] = DiameterTemp[0] + 0x30; //公共接头 EPC 一位直径直接显示原始整数。
			  dat[15] = 0x20; //公共接头 EPC 一位直径后清空旧小数点位置。
			  dat[16] = 0x20; //公共接头 EPC 一位直径后清空旧小数位位置。
		  }
	  }
	  else
	  {
		  if(DiameterTemp[2] > 0U)
		  {
			  diameter_decimal_extra = 1U; /* Page3 直径按 0.1mm 存储，150 必须显示为 15.0，不能丢掉百位。 */
			  dat[2] = 32; /* 直径从 x.y 扩展为 xx.y 后，DWIN 本帧数据长度增加 1 字节。 */
			  dat[14] = DiameterTemp[2] + 0x30; //直径十位，150 中的 1 表示 15.0 的十位。
			  dat[15] = DiameterTemp[1] + 0x30; //直径个位，150 中的 5 表示 15.0 的个位。
			  dat[16] = 0x2E;  //.
			  dat[17] = DiameterTemp[0] + 0x30; //直径小数位，151 时这里显示 1。
		  }
		  else
		  {
			  dat[14] = DiameterTemp[1] + 0x30; //直径整数位，40 显示为 4.0 时保持旧位置。
			  dat[15] = 0x2E;  //.
			  dat[16] = DiameterTemp[0] + 0x30; //直径小数位，40 显示为 4.0。
		  }
	  }
	  dat[17 + diameter_decimal_extra] = 0xA1;
	  dat[18 + diameter_decimal_extra] = 0xA2;  //、
	  dat[19 + diameter_decimal_extra] = 0x20;  //空格

	  dat[20 + diameter_decimal_extra] = LengthTemp[2] + 0x30;
	  dat[21 + diameter_decimal_extra] = LengthTemp[1] + 0x30;
	  dat[22 + diameter_decimal_extra] = LengthTemp[0] + 0x30;
	  dat[23 + diameter_decimal_extra] = 0x6D;  //'m'
	  dat[24 + diameter_decimal_extra] = 0x6D;  //'m'
	  dat[25 + diameter_decimal_extra] = 0xA1;
	  dat[26 + diameter_decimal_extra] = 0xA2;  //、
	  dat[27 + diameter_decimal_extra] = 0x20;

	  if(AngleTemp[2] > 0)//百位不为0
	  {
   	  dat[28 + diameter_decimal_extra] = AngleTemp[2] + 0x30;
	    dat[29 + diameter_decimal_extra] = AngleTemp[1] + 0x30;
	    dat[30 + diameter_decimal_extra] = AngleTemp[0] + 0x30;
	    dat[31 + diameter_decimal_extra] = 0xA1;
	    dat[32 + diameter_decimal_extra] = 0xE3;  //°
	    dat[33 + diameter_decimal_extra] = 0x20;
	  }
	  else if(AngleTemp[1] > 0)//百位为0，十位不为0
	  {
	    dat[28 + diameter_decimal_extra] = AngleTemp[1] + 0x30;
	    dat[29 + diameter_decimal_extra] = AngleTemp[0] + 0x30 ;
	    dat[30 + diameter_decimal_extra] = 0xA1;
	    dat[31 + diameter_decimal_extra] = 0xE3;  //°
	    dat[32 + diameter_decimal_extra] = 0x20;
	    dat[33 + diameter_decimal_extra] = 0x20;
	  }
	  else //百位为0，十位为0
	  {
	    dat[28 + diameter_decimal_extra] = AngleTemp[0] + 0x30 ;
	    dat[29 + diameter_decimal_extra] = 0xA1;
	    dat[30 + diameter_decimal_extra] = 0xE3;  //°
	    dat[31 + diameter_decimal_extra] = 0x20;
	    dat[32 + diameter_decimal_extra] = 0x20;
	    dat[33 + diameter_decimal_extra] = 0x20;
	  }
  }

  Uart6_SendPacket(dat, (uint16_t)(dat[2] + 3U));

}

/*
 * 函数功能：按公共接头 EPC 原始整数格式刷新主界面刀具规格。
 * 输入参数：Addr 为 DWIN 规格文本地址；Length 为长度原始整数；Diameter 为直径原始整数；Angle 为角度原始整数。
 * 返回参数：无。
 */
void LCD_IntegratedCutterRawData_Update(uint16_t Addr, uint16_t Length, uint8_t Diameter, uint8_t Angle)
{
  s_lcd_integrated_cutter_raw_integer_mode = 1U; /* 本次下发启用公共接头 EPC 原始直径格式，直径 16 显示为 Φ16。 */
  LCD_IntegratedCutterData_Update(Addr, Length, Diameter, Angle); /* 复用同一条 DWIN 文本组包路径，只切换直径字符格式。 */
  s_lcd_integrated_cutter_raw_integer_mode = 0U; /* 下发完成立即恢复旧格式，避免 PXBA/PXBB 下一次刷新误用整数直径。 */
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


