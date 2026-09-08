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

/*
 * 函数功能：尝试释放被从设备拉低的软件 IIC 总线，使后续 EEPROM 访问可以重新产生起始信号。
 * 输入参数：无。
 * 返回参数：无。
 */
void IIC_Reset(void)
{
  uint8_t i;
  SDA_H(); /* 主机先释放 SDA，随后读取的低电平才表示从设备仍占用总线。 */
  Delay_us(10); /* 给 GPIO 开漏线留出稳定时间，避免刚切换方向就误判总线状态。 */

  if(SDA_STAT == 0)
  {
	  /* SDA 被从设备卡低时补发 9 个 SCL，推动从设备移出残留字节并释放总线。 */
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

/*
 * 函数功能：等待从设备在第 9 个时钟拉低 SDA 应答，超时后主动停止本次事务。
 * 输入参数：无。
 * 返回参数：0 表示收到 ACK；1 表示等待超时并已发送停止信号。
 */
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
	    Signal_Stop(); /* 从设备长时间不应答时立即释放总线，避免后续事务一直处于占用状态。 */
      ack = 1; /* 把本次事务标记为失败，由上层决定重试或上报 EEPROM 异常。 */
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

/*
 * 函数功能：按高位在前的 IIC 时序发送一个字节，ACK 由调用方随后单独检查。
 * 输入参数：wdata 为待发送字节。
 * 返回参数：固定返回 1，保持旧调用接口不变。
 */
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
	    SDA_H(); /* 当前最高位为 1 时释放 SDA，由上拉电阻形成高电平。 */
	  }
	  else
	  {
	    SDA_L(); /* 当前最高位为 0 时由主机主动拉低 SDA。 */
	  }

	  Delay_us(10);

	  SCL_H();
	  Delay_us(10);

	  mdata <<= 1;
	  SCL_L();
  }

	return 1;
}

/*
 * 函数功能：按高位在前的 IIC 时序读取一个字节，并在字节末尾发送 ACK 或 NACK。
 * 输入参数：ack 为 1 时继续连续读取并发送 ACK，为 0 时结束读取并发送 NACK。
 * 返回参数：本次从 SDA 采样得到的 8 位数据。
 */
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

	  /* SCL 拉高后的 SDA 为高表示当前接收位为 1，需要写入结果最低位。 */
	  if(SDA_STAT > 0)
	    rdata++;

	  Delay_us(5);
  }

  (ack == 1) ? IIC_Ack() : IIC_NAck(); /* 非最后字节用 ACK 请求继续发送，最后字节用 NACK 结束连续读。 */

  return rdata;
}

//============================================================================
//AT24C02读写测试，会覆盖0xFA~0xFC的原数据，并且不会恢复。
//============================================================================
/*
 * 函数功能：写入3个测试字节再读回比较，用于检查旧EEPROM单字节接口。
 * 输入参数：无。
 * 返回参数：1表示3字节全部一致，0表示至少一个字节不一致；测试地址原内容不会恢复。
 */
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
		/* 读回任一字节不同都判定 EEPROM 自检失败，避免接受部分写入的数据。 */
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

  /* 容量大于 AT24C16 的器件使用 16 位内部地址，需要先发送地址高字节。 */
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
/*
 * 函数功能：通过旧接口写入一个EEPROM字节，地址发送方式由EE_TYPE选择。
 * 输入参数：WriteAddr为字节地址，DataToWrite为待写字节。
 * 返回参数：无；本函数未检查各次IIC_WaitAck的结果，返回不代表已确认写入成功。
 */
void IIC_AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite)
{
  Signal_Start();

  /* 型号容量大于AT24C16时，写入前先发送16位内部地址的高字节。 */
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
static uint8_t Iic_WriteAt24Chunk(uint16_t WriteAddr, const uint8_t *pBuffer, uint16_t NumToWrite)
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

    if (Iic_WriteAt24Chunk(WriteAddr, pBuffer, write_len) == 0U)
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















