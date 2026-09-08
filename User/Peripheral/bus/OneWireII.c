//OneWireII.c

#include "OneWireII.h"
#include "bsp_gpio.h"
#include "delay.h"
#include "data.h"

#define ONEWIREII_H()    Bsp_GpioWrite(BOARD_RES_ONEWIRE_II_PORT, BOARD_RES_ONEWIRE_II_PIN, GPIO_PIN_SET)
#define ONEWIREII_L()    Bsp_GpioWrite(BOARD_RES_ONEWIRE_II_PORT, BOARD_RES_ONEWIRE_II_PIN, GPIO_PIN_RESET)

#define ONEWIREII_STAT   Bsp_GpioRead(BOARD_RES_ONEWIRE_II_PORT, BOARD_RES_ONEWIRE_II_PIN)

//1-Wire
//============================================================================
//初始化时序（复位+从机应答）
//主机通过拉低单总线480 ~ 960 us产生复位脉冲，然后释放总线，进入接收模式。
//主机释放总线时，会产生低电平跳变为高电平的上升沿，单总线器件检测到上升沿之后，
//延时15 ~ 60 us，单总线器件拉低总线60 ~ 240 us来产生应答脉冲。
//主机接收到从机的应答脉冲说明单总线器件就绪，初始化过程完成。
//============================================================================
/*
 * 函数功能：向 II 路 GX2431 发送复位脉冲，并检查从设备是否返回低电平存在脉冲。
 * 输入参数：无。
 * 返回参数：0 表示检测到从设备；1 表示总线保持高电平、设备未应答。
 */
uint8_t OneWireII_Reset(void)
{
	uint8_t ret = 0;

  ONEWIREII_L();   //产生复位信号...

  Delay_us(500);  //480 ~ 960us

  ONEWIREII_H();    //释放总线

  Delay_us(70);

  /* 采样点为高电平表示从设备返回位 1；低电平保持默认位 0。 */
  if (ONEWIREII_STAT > 0)
		ret = 1; /* 70us 采样点仍为高电平，说明从设备没有拉低存在脉冲，本次访问不可继续。 */

	Delay_us(430);

	return ret;
}

//============================================================================
//============================================================================
 void OneWireII_SendBit1(void)
{
  ONEWIREII_L();

  Delay_us(5);

  ONEWIREII_H();

  Delay_us(65);
}

//============================================================================
//============================================================================
 void OneWireII_SendBit0(void)
{
  ONEWIREII_L();

  Delay_us(65);

  ONEWIREII_H();

  Delay_us(5);
}

//============================================================================
//读
//主机拉低总线至少1us后释放，并在本次读时隙开始后的15us内读取电平。
//============================================================================
/*
 * 函数功能：通过II路单总线读取1位，拉低3us、释放后等待7us，再读取电平。
 * 输入参数：无。
 * 返回参数：1表示读到高电平，0表示低电平。
 */
uint8_t OneWireII_ReceiveBit(void)
{
  uint8_t bit = 0;

  ONEWIREII_L();

  Delay_us(3);

  ONEWIREII_H();

  Delay_us(7);


  /* 释放总线后采样为高电平时，本次接收位按 1 返回；低电平保持初值 0。 */
  if (ONEWIREII_STAT > 0)
		bit = 1;

  Delay_us(60);

  return bit;
}

//============================================================================
//============================================================================
void OneWireII_SendByte(uint8_t byte)
{
  uint8_t i = 0;
  uint8_t tmpByte = byte;

  for (i = 0; i < 8; i++)
  {
    /* 当前待发位为 1 时使用短低脉冲时隙，否则使用写 0 的长低脉冲。 */
    if (tmpByte & (0x01 << i))
	    OneWireII_SendBit1();
	  else
      OneWireII_SendBit0();
  }
}

//============================================================================
//============================================================================
uint8_t OneWireII_ReceiveByte(void)
{
  uint8_t i = 0, Tbit = 0;
  uint8_t byte = 0;

  for (i = 0; i < 8; i++)
  {
	  Tbit = OneWireII_ReceiveBit();
	  /* 采样到位 1 时写入当前位位置，位 0 保持结果字节原值。 */
	  if (Tbit)
	    byte |= (0x01 << i);
  }

  return byte;
}

//============================================================================
//============================================================================
//*@函数描述: 读8位家族码;48位序列号;8位CRC码;一共8个字节
//*@输入参数: id数据缓存
//*@输出参数: 无
//*@返 回 值: 1 总线不可以；0 正常；2 CRC检验失败
//*@其他说明:	GX2431 有两种不同类型的CRC码，如下：
//								一种为8位，存储在64位ROM的最高字节中。主机能根
//						据64位ROM码的前56位计算出该CRC码，并与存储在GX2431 中的值比较，判断ROM数据是
//						否接收无误。计算该CRC校验码的等效多项式为：X8 + X5 + X4 + 1。接收到的8位CRC为
//						原码（不取反）形式。该值在工厂计算并被光刻写入ROM中。
//								另一种CRC码为16位，采用标准的CRC16多项式函数：X16 + X15  + X2  + 1 产生。
//						该CRC校验码用来对读写暂存器时传输的数据进行快速校验。与8位CRC校验码不同，16位
//						CRC校验码总是以反码的形式传输。
//*@注意事项:	Read ROM--此命只用于读取GX2431的8位家族码，唯一的48ID号和8位CRC校验码。
//*@历    史：

//*@		    说明：
//功能：读8位家族码;48位序列号;8位CRC码;读取成功返回0
//参数：*id--读取的数据存放地址
//返回：0--操作成功；1--总线不可用;
//============================================================================
void OneWireII_DS2431_ReadRom(uint8_t *id)
{
  uint8_t i = 0;

  OneWireII_Reset();

  OneWireII_SendByte(Rom_Read_Cmd);        //写命令

  for (i = 0; i < 8; i++)
    *id++ = OneWireII_ReceiveByte();
}

//============================================================================
//============================================================================
//功能：  读EPROM
//参数：  tgaddr--目标地址;0~128
//        len--要读取的字节数;
//       *buffer--存放地址
//返回：0--操作成功；1--总线不可用;
//用时：1ms + n * 560us ≈ 12.2ms
//============================================================================
uint8_t OneWireII_DS2431_ReadMemory(uint8_t tgaddr, uint8_t len, uint8_t *buffer)
{
  uint8_t i = 0;                                                                                                                                                                                                                                                                                                                            

	ONEWIREII_L();//DQ_OUT = 0;
	Delay_ms( 20 );  //复位低脉冲保持 
  ONEWIREII_H();//DQ_OUT = 1;	
  Delay_ms( 1 );  //复位低脉冲保持 	
	
  if (OneWireII_Reset())  //1ms
	  return 1; /* 复位后没有收到存在脉冲，停止读存储器，避免把空总线高电平当成有效数据。 */

  OneWireII_SendByte(Rom_Skip_Cmd);      //写命令 560us
  OneWireII_SendByte(Memory_Read_Cmd);    //写命令 560us
  OneWireII_SendByte((tgaddr << 5) & 0xff); //( tgaddr );  //写地址低字节 560us
  OneWireII_SendByte(0);                 //写地址高字节 560us

  for (i = 0; i < len; i++)
	  buffer[i] = OneWireII_ReceiveByte();  //560us

  return 0;
}

//============================================================================
//============================================================================
//描述：DS2431的EEPROM 共为8 X 18个字节, 可以看作18个8字节块.
//功能：写EPROM
//参数：nblock--块号取值( 0--17 )16块为特殊功能寄存器17块为保留;
//      *buffer为要写入的8字节数据起始指针
//返回：  0--操作成功；
//        1--总线不可用；
//        2--写暂存器失败;
//        3--写主存储器错误;
//============================================================================
/*
 * 函数功能：向 II 路 GX2431 指定 8 字节块写入数据，读回暂存器校验后再复制到 EEPROM。
 * 输入参数：nblock 为 0~17 块号；buffer 指向待写入的 8 字节数据。
 * 返回参数：0 成功；1 总线不可用；2 暂存器或授权码校验失败；3 地址或复制失败。
 */
uint8_t OneWireII_DS2431_WriteMemory(uint8_t nblock, uint8_t *buffer )
{
  uint8_t sbuf[16] = { 0 };
  uint8_t i, TA1, TA2, E_S;

  if (nblock > 17)
    return 3; /* GX2431 只允许 0~17 块，越界时禁止继续发送写命令，避免地址回卷。 */

  OneWireII_Reset();

  OneWireII_SendByte(0xCC );
  OneWireII_SendByte(0x0F );
  OneWireII_SendByte(nblock * 8);
  OneWireII_SendByte(0x00);

  for (i = 0; i < 8; i++)
    OneWireII_SendByte(buffer[i]);

  OneWireII_ReceiveByte();
  OneWireII_ReceiveByte();
  Delay_us(200);

  OneWireII_Reset();

  OneWireII_SendByte(0xCC);
  OneWireII_SendByte(0xAA);

  /*获取授权码*/
  TA1 = OneWireII_ReceiveByte();
  TA2 = OneWireII_ReceiveByte();
  E_S = OneWireII_ReceiveByte();

  /*校验授权码*/
  if (TA1 != ( nblock * 8))
	  return 2; /* TA1 必须等于目标块首地址，不一致说明暂存器写入地址已经错位。 */
  else if (TA2 != 0)  //TA2(always 0 for GX2431)
	  return 2; /* 当前 GX2431 地址高字节固定为 0，非零时不能把暂存器复制到目标块。 */
  else if (E_S != 7)  //E_S(always 7 for GX2431)
	  return 2; /* 写满 8 字节后结束偏移应为 7，异常值表示暂存器内容不完整。 */

  Delay_us(10);
  for (i = 0; i < 8; i++)  //读8个数据
  {
    sbuf[i] = OneWireII_ReceiveByte();
	  if (sbuf[i] !=  buffer[i])
	    return 2; /* 暂存器任一字节与待写数据不一致时拒绝复制，防止错误数据写入 EEPROM。 */
  }

  //crc字节读取，但是没进行校验，需要自己实现CRC16多项式函数：X16 + X15  + X2  + 1
  OneWireII_ReceiveByte();
  OneWireII_ReceiveByte();

  OneWireII_Reset();

  OneWireII_SendByte(0xCC); // Send Skip ROM command to select single device
  OneWireII_SendByte(0x55); // Read Authentication command

  //发送授权码
  OneWireII_SendByte(TA1);
  OneWireII_SendByte(TA2);
  OneWireII_SendByte(E_S);

  Delay_ms(15);  //延时很重要，等待tPROGmax ，完成复制操作
  if (OneWireII_ReceiveByte() == 0xaa)
    return 0; /* 0xAA 是 GX2431 复制完成确认码，收到后才判定 EEPROM 写入成功。 */
  else
	  return 3; /* 未收到完成确认码时保留失败状态，避免上层误认为数据已经落盘。 */
}

//============================================================================
//============================================================================


//============================================================================
//============================================================================


//============================================================================
//============================================================================


//============================================================================
//============================================================================















