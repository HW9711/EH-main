//iic.c

#include "stm32f4xx_hal.h"
#include "iic.h"
#include "delay.h"

#define SCL_H()    GPIOC->BSRR = GPIO_PIN_4
#define SCL_L()    GPIOC->BSRR = (uint32_t)GPIO_PIN_4 << 16U

#define SDA_H()    GPIOC->BSRR = GPIO_PIN_5
#define SDA_L()    GPIOC->BSRR = (uint32_t)GPIO_PIN_5 << 16U

#define SDA_STAT   (GPIOC->IDR & GPIO_PIN_5)

//IIC
//============================================================================
//============================================================================
void IIC_Init(void)
{
  SDA_H();
  SCL_H();
}

//============================================================================
//============================================================================
void IIC_Reset(void)
{
  uint8_t i;
  SDA_H();
  Delay_us(10);

  if(SDA_STAT == 0)
  {
	  for(i = 0; i < 9; i ++)
	  {
	    SCL_H();
	    Delay_ms(1);
	    SCL_L();
	    Delay_ms(1);
	  }
  }
}

//============================================================================
//============================================================================
 static void Signal_Start(void)
{
  SDA_H();
	SCL_H();
  Delay_us(10);

  SDA_L();
  Delay_us(10);

  SCL_L();
  Delay_us(10);
}

//============================================================================
//============================================================================
static void Signal_Stop(void)
{
  SCL_L();
  SDA_L();
  Delay_us(10);

  SCL_H();
  Delay_us(10);

  SDA_H();
  Delay_us(10);
}

//============================================================================
//============================================================================
uint8_t IIC_WaitAck(void)
{
  uint8_t errCount = 0;
  uint8_t ack = 0;

  SCL_H();
  Delay_us(10);

  while(SDA_STAT > 0)
  {
    errCount++;
    if(errCount > 200)
	  {
	    Signal_Stop();
      ack = 1;
      break;
    }
  }

  SCL_L();
  Delay_us(10);

  return ack;
}

//============================================================================
//============================================================================
void IIC_Ack(void)
{
  SCL_L();
  SDA_L();
  Delay_us(10);

  SCL_H();
  Delay_us(10);

  SCL_L();
  Delay_us(10);
}

//============================================================================
//============================================================================
void IIC_NAck(void)
{
  SCL_L();
  SDA_H();
  Delay_us(10);

  SCL_H();
  Delay_us(10);

  SCL_L();
  Delay_us(10);
}

//============================================================================
//============================================================================
static uint8_t Write_Byte(uint8_t wdata)
{
  uint8_t i,mdata;

  mdata = wdata;

  SCL_L();
  for(i = 0; i < 8; i++)
  {
		Delay_us(10);
	  if(mdata & 0x80)
	  {
	    SDA_H();
	  }
	  else
	  {
	    SDA_L();
	  }

	  Delay_us(10);

	  SCL_H();
	  Delay_us(10);

	  mdata <<= 1;
	  SCL_L();
  }

	return 1;
}

//============================================================================
//============================================================================
static uint8_t Read_Byte(uint8_t ack)
{
  uint8_t i,rdata = 0;

  for(i = 0; i < 8; i++)
  {
	  rdata <<= 1;

		SCL_L();
	  Delay_us(10);

	  SCL_H();
	  Delay_us(5);

	  if(SDA_STAT > 0)
	    rdata++;

	  Delay_us(5);
  }

  (ack == 1) ? IIC_Ack() : IIC_NAck();

  return rdata;
}

//============================================================================
//AT24C02 读写测试
//============================================================================
uint8_t IIC_WriteRead_Test(void)
{
	uint8_t i = 0;
	uint8_t temp[3] = {0x55, 0xeb, 0x90}, temp1[3] = { 0 };

	for (i = 0; i < 3; i++)
	  IIC_AT24CXX_WriteOneByte(0xFA + i, temp[i]);

	for (i = 0; i < 3; i++)
	  temp1[i] = IIC_AT24CXX_ReadOneByte(0xFA + i);

	for (i = 0; i < 3; i++)
	{
		if (temp[i] != temp1[i])
			return 0;
	}

	return 1;	
}

//============================================================================
//============================================================================
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址
//返回值  :读到的数据
uint8_t IIC_AT24CXX_ReadOneByte(uint16_t ReadAddr)
{
  uint8_t temp = 0;

  Signal_Start();

  if (EE_TYPE > AT24C16)
  {
	  Write_Byte(0xA0);	   //发送写命令
	  IIC_WaitAck();
	  Write_Byte(ReadAddr >> 8); //发送高地址
  }
  else
	  Write_Byte(0xA0 + ((ReadAddr / 256) << 1));   //发送器件地址0XA0,写数据

  IIC_WaitAck();
  Write_Byte(ReadAddr % 256);   //发送低地址

  IIC_WaitAck();
  Signal_Start();

  Write_Byte(0xA1);           //进入接收模式
  IIC_WaitAck();

  temp = Read_Byte(0);

  Signal_Stop(); //产生一个停止条件
  Delay_ms(1);

  return temp;
}

//============================================================================
//============================================================================
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void IIC_AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite)
{
  Signal_Start();

  if (EE_TYPE > AT24C16)
  {
	  Write_Byte(0xA0);	    //发送写命令
	  IIC_WaitAck();
	  Write_Byte(WriteAddr >> 8);  //发送高地址
  }
  else
	  Write_Byte(0xA0 + ((WriteAddr / 256) << 1));   //发送器件地址0XA0,写数据

  IIC_WaitAck();
  Write_Byte(WriteAddr % 256);   //发送低地址

  IIC_WaitAck();
  Write_Byte(DataToWrite);     //发送字节

  IIC_WaitAck();
  Signal_Stop();  //产生一个停止条件

  Delay_ms(1);
}

//============================================================================
//============================================================================


//============================================================================
//============================================================================















