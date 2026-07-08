//iic.c

#include "iic.h"
#include "bsp_gpio.h"
#include "delay.h"

#define SCL_H()    Bsp_GpioWrite(BOARD_RES_SOFT_IIC_SCL_PORT, BOARD_RES_SOFT_IIC_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()    Bsp_GpioWrite(BOARD_RES_SOFT_IIC_SCL_PORT, BOARD_RES_SOFT_IIC_SCL_PIN, GPIO_PIN_RESET)

#define SDA_H()    Bsp_GpioWrite(BOARD_RES_SOFT_IIC_SDA_PORT, BOARD_RES_SOFT_IIC_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()    Bsp_GpioWrite(BOARD_RES_SOFT_IIC_SDA_PORT, BOARD_RES_SOFT_IIC_SDA_PIN, GPIO_PIN_RESET)

#define SDA_STAT   Bsp_GpioRead(BOARD_RES_SOFT_IIC_SDA_PORT, BOARD_RES_SOFT_IIC_SDA_PIN)

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

  SDA_H(); /* 主机发送完 8 位后必须释放 SDA，由 AT24C32 拉低应答；否则末位为 0 的字节会被主机误判成 ACK。 */
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

  SDA_H(); /* 读数据阶段主机必须释放 SDA，特别是连续读 ACK 后要让 EEPROM 重新驱动下一字节。 */
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

/*
 * 函数功能：按 AT24C32 16 位字地址连续读取主控板 EEPROM 数据。
 * 输入参数：ReadAddr 为 AT24C32 内部起始地址，pBuffer 为接收缓存，NumToRead 为读取字节数。
 * 返回参数：1 表示读取成功，0 表示参数非法或 EEPROM 未应答。
 */
uint8_t IIC_AT24C32_ReadBytes(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
  uint16_t index; /* 当前读取字节下标，用于决定最后一个字节后发送 NACK。 */

  if ((pBuffer == 0) && (NumToRead != 0U))
  {
    return 0U; /* 有读取长度但缓存为空时直接拒绝，避免破坏内存。 */
  }

  if (NumToRead == 0U)
  {
    return 1U; /* 0 长度读取视为成功，方便上层复用边界逻辑。 */
  }

  Signal_Start(); /* 先进入写地址阶段，把 AT24C32 内部地址指针设到 ReadAddr。 */

  Write_Byte(AT24C32_IIC_WRITE_ADDR); /* AT24C32 A0/A1/A2 接地，写阶段设备地址固定 0xA0。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 设备地址未应答，说明板载 EEPROM 不在线或总线异常。 */
  }

  Write_Byte((uint8_t)(ReadAddr >> 8)); /* AT24C32 容量超过 16Kbit，内部地址必须先发高字节。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 高地址未应答时停止本次事务，避免继续读到错误位置。 */
  }

  Write_Byte((uint8_t)(ReadAddr & 0xFFU)); /* 再发低字节，完成 16 位内部地址装载。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 低地址未应答时停止本次事务。 */
  }

  Signal_Start(); /* 重复起始进入读阶段，保持 EEPROM 内部地址指针不被释放。 */
  Write_Byte(AT24C32_IIC_READ_ADDR); /* 读阶段设备地址固定 0xA1。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 读地址未应答时停止，避免上层使用无效缓存。 */
  }

  for (index = 0U; index < NumToRead; ++index)
  {
    pBuffer[index] = Read_Byte((index + 1U) < NumToRead ? 1U : 0U); /* 非最后字节 ACK，最后字节 NACK 结束连续读。 */
  }

  Signal_Stop(); /* 连续读完成后释放总线，避免影响后续手柄/主控初始化流程。 */
  Delay_ms(1); /* EEPROM 读后留最小间隔，保持和旧接口时序风格一致。 */
  return 1U;
}

/*
 * 函数功能：按 AT24C32 16 位字地址写入一段不跨页的数据。
 * 输入参数：WriteAddr 为 AT24C32 内部起始地址，pBuffer 为待写数据，NumToWrite 为本次写入字节数。
 * 返回参数：1 表示写入事务已被 EEPROM 应答，0 表示参数非法或 EEPROM 未应答。
 */
static uint8_t IIC_AT24C32_WriteChunk(uint16_t WriteAddr, const uint8_t *pBuffer, uint16_t NumToWrite)
{
  uint16_t index; /* 当前写入字节下标，用于逐字节等待 EEPROM ACK。 */

  if (((pBuffer == 0) && (NumToWrite != 0U)) || (NumToWrite > AT24C32_IIC_PAGE_SIZE))
  {
    return 0U; /* 缓存非法或单次超过页大小时拒绝，防止 AT24C32 页内回卷覆盖。 */
  }

  Signal_Start(); /* 写入从起始信号开始，占用 PC4/PC5 软件 IIC 总线。 */

  Write_Byte(AT24C32_IIC_WRITE_ADDR); /* AT24C32 写设备地址固定 0xA0。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 器件未应答时停止事务，让上层记录 EEPROM 异常。 */
  }

  Write_Byte((uint8_t)(WriteAddr >> 8)); /* 写入 16 位内部地址高字节。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 高地址未应答，停止事务避免地址错位。 */
  }

  Write_Byte((uint8_t)(WriteAddr & 0xFFU)); /* 写入 16 位内部地址低字节。 */
  if (IIC_WaitAck() != 0U)
  {
    Signal_Stop();
    return 0U; /* 低地址未应答，停止事务。 */
  }

  for (index = 0U; index < NumToWrite; ++index)
  {
    Write_Byte(pBuffer[index]); /* 按页写事务连续发送数据字节，减少 Page1 写入耗时。 */
    if (IIC_WaitAck() != 0U)
    {
      Signal_Stop();
      return 0U; /* 任一数据字节未应答都视为写入失败，避免记录半页版本号。 */
    }
  }

  Signal_Stop(); /* 停止信号触发 AT24C32 内部写周期。 */
  Delay_ms(5); /* AT24C32 典型写周期需要数毫秒，等待完成后再允许读回校验。 */
  return 1U;
}

/*
 * 函数功能：按 AT24C32 页边界连续写入主控板 EEPROM 数据。
 * 输入参数：WriteAddr 为 AT24C32 内部起始地址，pBuffer 为待写缓存，NumToWrite 为写入字节数。
 * 返回参数：1 表示全部写入成功，0 表示参数非法或某个分页写失败。
 */
uint8_t IIC_AT24C32_WriteBytes(uint16_t WriteAddr, const uint8_t *pBuffer, uint16_t NumToWrite)
{
  uint16_t write_len; /* 本轮实际写入长度，保证不跨 32 字节页边界。 */
  uint16_t page_left; /* 当前地址到本页页尾还剩多少字节。 */

  if ((pBuffer == 0) && (NumToWrite != 0U))
  {
    return 0U; /* 有写入长度但数据为空时拒绝，防止写入随机数据。 */
  }

  while (NumToWrite != 0U)
  {
    page_left = (uint16_t)(AT24C32_IIC_PAGE_SIZE - (WriteAddr % AT24C32_IIC_PAGE_SIZE)); /* 计算当前页剩余容量。 */
    write_len = (NumToWrite > page_left) ? page_left : NumToWrite; /* 单次写入不能跨页，否则 AT24C32 会回卷覆盖本页前部。 */

    if (IIC_AT24C32_WriteChunk(WriteAddr, pBuffer, write_len) == 0U)
    {
      return 0U; /* 任一分页写失败时立即返回，让上层保留旧版本状态。 */
    }

    WriteAddr = (uint16_t)(WriteAddr + write_len); /* 推进 EEPROM 地址，下一轮从新页或页内后续位置继续。 */
    pBuffer += write_len; /* 推进待写缓存指针，避免重复写同一段数据。 */
    NumToWrite = (uint16_t)(NumToWrite - write_len); /* 扣除已成功写入长度。 */
  }

  return 1U;
}


//============================================================================
//============================================================================















