
#include "eeprom.h"
#include "iic.h"

//初始化IIC接口
void EEPROM_AT24CXX_Init(void)
{
  IIC_Init();
}

//在AT24CXX里面的指定地址开始写长度为NumToWrite的数据
//ReadAddr   :开始写入的地址
//*pBuffer   :数据
//NumToWrite :要写入数据的长度
void EEPROM_AT24CXX_Write(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
  while(NumToWrite)
  {
	  IIC_AT24CXX_WriteOneByte(ReadAddr++, *pBuffer++);
	  NumToWrite--;
  }
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void EEPROM_AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
  while(NumToRead)
  {
	  *pBuffer++ = IIC_AT24CXX_ReadOneByte(ReadAddr++);
	  NumToRead--;
  }
}

