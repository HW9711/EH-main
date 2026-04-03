//OneWireI.c

#include "stm32f4xx_hal.h"
#include "OneWireI.h"
#include "delay.h"
#include "data.h"

#define ONEWIREI_H()    GPIOD->BSRR = GPIO_PIN_13
#define ONEWIREI_L()    GPIOD->BSRR = (uint32_t)GPIO_PIN_13 << 16U

#define ONEWIREI_STAT   HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)

//1-Wire
//============================================================================
//1. 复位/应答脉冲
//复位/应答脉冲要求：主机拉低总线 480~960us 来产生复位信号，然后释放总线进入接收模式，
//接着从机等待 15-60us（上拉电阻上拉至高电平），从机再拉低总线 60-240us 来产生应答信号，
//主机接收到从机的应答信号后，表明从机准备就绪，初始化过程完成了。
//============================================================================
uint8_t OneWireI_Reset(void)
{
	uint8_t ret = 0;

  ONEWIREI_L();   //产生复位信号...

  Delay_us(500);  //480 ~ 960us

  ONEWIREI_H();    //释放总线

  Delay_us(70);

	if (ONEWIREI_STAT > 0)
		ret = 1;

	Delay_us(430);

	return ret;
}

//============================================================================
//3. 写 1 时隙
//写1时隙要求：写 1 时隙和写 0 时隙一样，起始于主机拉低总线，
//在拉低总线 15us 之内需要将总线拉高，拉高总线需要维持 60us 以上。
//============================================================================
 void OneWireI_SendBit1(void)
{
  ONEWIREI_L();

  Delay_us(5);

  ONEWIREI_H();

  Delay_us(65);
}

//============================================================================
//2. 写 0 时隙
//写0时隙要求：写 0 时隙起始于主机拉低总线，主机拉低总线后，
//只需在整个时隙期间保持总线低电平在 60us 以上，一般是 60~120us 之间。
//============================================================================
 void OneWireI_SendBit0(void)
{
  ONEWIREI_L();

  Delay_us(65);

  ONEWIREI_H();

  Delay_us(5);
}

//============================================================================
//读
//当主机把总线拉低时，并保持至少1 us后释放总线，必须在15 us内读取数据。
//============================================================================
uint8_t OneWireI_ReceiveBit(void)
{
  uint8_t bit = 0;

  ONEWIREI_L();

  Delay_us(3);

  ONEWIREI_H();

  Delay_us(7);

  if (ONEWIREI_STAT > 0)
		bit = 1;

  Delay_us(60);

  return bit;
}

//============================================================================
//============================================================================
void OneWireI_SendByte(uint8_t byte)
{
  uint8_t i = 0;
  uint8_t tmpByte = byte;

  for (i = 0; i < 8; i++)
  {
    if (tmpByte & (0x01 << i))
      OneWireI_SendBit1();
	  else
      OneWireI_SendBit0();
  }
}

//============================================================================
//============================================================================
uint8_t OneWireI_ReceiveByte(void)
{
  uint8_t i = 0, Tbit = 0;
  uint8_t byte = 0;

  for (i = 0; i < 8; i++)
  {
	  Tbit = OneWireI_ReceiveBit();
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
void OneWireI_DS2431_ReadRom(uint8_t *id)
{
  uint8_t i = 0;

  OneWireI_Reset();

  OneWireI_SendByte(Rom_Read_Cmd);        //写命令

  for (i = 0; i < 8; i++)
    *id++ = OneWireI_ReceiveByte();
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
uint8_t OneWireI_DS2431_ReadMemory(uint8_t tgaddr, uint8_t len, uint8_t *buffer)
{
  uint8_t i = 0;

	
  ONEWIREI_L();//DQ_OUT = 0;
	Delay_ms( 20 );  //复位低脉冲保持 
  ONEWIREI_H();//DQ_OUT = 1;	
  Delay_ms( 1 );  //复位低脉冲保持 	
	
	
  if (OneWireI_Reset())  //1ms
	  return 1;

  OneWireI_SendByte(Rom_Skip_Cmd);      //写命令 560us
  OneWireI_SendByte(Memory_Read_Cmd);    //写命令 560us
  OneWireI_SendByte((tgaddr << 5) & 0xff); //( tgaddr );  //写地址低字节 560us
  OneWireI_SendByte(0);                 //写地址高字节 560us

  for (i = 0; i < len; i++)
	  buffer[i] = OneWireI_ReceiveByte();  //560us

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
uint8_t OneWireI_DS2431_WriteMemory(uint8_t nblock, uint8_t *buffer )
{
  uint8_t sbuf[16] = { 0 };
  uint8_t i, TA1, TA2, E_S;

  if (nblock > 17)
    return 3;                                //地址超出范围

  OneWireI_Reset();

  OneWireI_SendByte(0xCC );
  OneWireI_SendByte(0x0F );
  OneWireI_SendByte(nblock * 8);
  OneWireI_SendByte(0x00);

  for (i = 0; i < 8; i++)
    OneWireI_SendByte(buffer[i]);

  OneWireI_ReceiveByte();
  OneWireI_ReceiveByte();
  Delay_us(200);

  OneWireI_Reset();

  OneWireI_SendByte(0xCC);
  OneWireI_SendByte(0xAA);

  /*获取授权码*/
  TA1 = OneWireI_ReceiveByte();
  TA2 = OneWireI_ReceiveByte();
  E_S = OneWireI_ReceiveByte();

  /*校验授权码*/
  if (TA1 != ( nblock * 8))
	  return 2;
  else if (TA2 != 0)  //TA2(always 0 for GX2431)
	  return 2;
  else if (E_S != 7)  //E_S(always 7 for GX2431)
	  return 2;

  Delay_us(10);
  for (i = 0; i < 8; i++)  //读8个数据
  {
    sbuf[i] = OneWireI_ReceiveByte();
	  if (sbuf[i] !=  buffer[i])
	    return 2;
  }

  //crc字节读取，但是没进行校验，需要自己实现CRC16多项式函数：X16 + X15  + X2  + 1
  OneWireI_ReceiveByte();
  OneWireI_ReceiveByte();

  OneWireI_Reset();

  OneWireI_SendByte(0xCC); // Send Skip ROM command to select single device
  OneWireI_SendByte(0x55); // Read Authentication command

  //发送授权码
  OneWireI_SendByte(TA1);
  OneWireI_SendByte(TA2);
  OneWireI_SendByte(E_S);

  Delay_ms(15);  //延时很重要，等待tPROGmax ，完成复制操作
  if (OneWireI_ReceiveByte() == 0xaa)
    return 0;
  else
	  return 3;
}

//============================================================================
//============================================================================


//============================================================================
//============================================================================


//============================================================================
//============================================================================


//============================================================================
//============================================================================















