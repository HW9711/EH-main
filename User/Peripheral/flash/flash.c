
#include "stm32f4xx_hal.h"
#include "flash.h"

//用户根据自己的需要设置
#define FLASH_SIZE   1024 	 		//所选STM32的FLASH容量大小(单位为K)

//FLASH起始地址
#define STM32_FLASH_BASE    0x08000000 	//STM32 FLASH的起始地址

//======================================================================================================
//获取操作扇区号
//start_address：操作地址
uint32_t Flash_GetSector_Index(uint32_t address)
{
  /* 地址低于扇区 1 起点时属于扇区 0。 */
  if (address < ADDR_FLASH_SERTOR_1)
	  return FLASH_SECTOR_0;
  /* 地址低于扇区 2 起点时属于扇区 1，后续判断按同一上边界规则递增。 */
  else if (address < ADDR_FLASH_SERTOR_2)
	  return FLASH_SECTOR_1;
  /* 地址落在扇区 2 的地址区间时返回扇区 2。 */
  else if (address < ADDR_FLASH_SERTOR_3)
	  return FLASH_SECTOR_2;
  /* 地址落在扇区 3 的地址区间时返回扇区 3。 */
  else if (address < ADDR_FLASH_SERTOR_4)
	  return FLASH_SECTOR_3;
  /* 地址落在扇区 4 的地址区间时返回扇区 4。 */
  else if (address < ADDR_FLASH_SERTOR_5)
	  return FLASH_SECTOR_4;
  /* 地址落在扇区 5 的地址区间时返回扇区 5。 */
  else if (address < ADDR_FLASH_SERTOR_6)
	  return FLASH_SECTOR_5;
  /* 地址落在扇区 6 的地址区间时返回扇区 6。 */
  else if (address < ADDR_FLASH_SERTOR_7)
	  return FLASH_SECTOR_6;
  /* 地址落在扇区 7 的地址区间时返回扇区 7。 */
  else if (address < ADDR_FLASH_SERTOR_8)
	  return FLASH_SECTOR_7;
  /* 地址落在扇区 8 的地址区间时返回扇区 8。 */
  else if (address < ADDR_FLASH_SERTOR_9)
	  return FLASH_SECTOR_8;
  /* 地址落在扇区 9 的地址区间时返回扇区 9。 */
  else if (address < ADDR_FLASH_SERTOR_10)
	  return FLASH_SECTOR_9;
  /* 地址落在扇区 10 的地址区间时返回扇区 10。 */
  else if (address < ADDR_FLASH_SERTOR_11)
    return FLASH_SECTOR_10;
  else
	  return FLASH_SECTOR_11;
}

//======================================================================================================
//======================================================================================================
//从指定地址开始写入指定长度的数据
int8_t Flash_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint16_t NumToWrite)
{
  FLASH_EraseInitTypeDef FlashEraseInit;
  uint32_t SectorError = 0;

	uint32_t StartWriteAddr = WriteAddr;  //写-起始地址
	uint32_t EndWriteAddr = WriteAddr + NumToWrite;  //写-结束地址

  uint32_t FirstSector = 0;     //起始扇区
  uint32_t NbOfSectors = 0;     //操作的扇区数量

	uint32_t TempAddr = 0, i = 0;  //当前操作地址

	/* 写入范围越过片内 Flash 边界时立即拒绝，避免擦除程序区外或无效地址。 */
  if ((StartWriteAddr < STM32_FLASH_BASE) || (EndWriteAddr >= (STM32_FLASH_BASE + FLASH_SIZE * 1024)))
	  return -1;

  //FLASH 解锁 ********************************
  //使能访问FLASH控制寄存器
  HAL_FLASH_Unlock();

  FirstSector = Flash_GetSector_Index(StartWriteAddr);
  NbOfSectors = Flash_GetSector_Index(EndWriteAddr)- FirstSector + 1;

  //擦除用户区域 (用户区域指程序本身没有使用的空间，可以自定义)
  //Fill EraseInit structure
	FlashEraseInit.Banks = FLASH_BANK_1;  //操作的扇区块
  FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; //擦除类型：标明Flash执行页面只做擦除操作
  FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; //以“字”的大小进行操作（电压范围）
  FlashEraseInit.Sector = FirstSector;
  FlashEraseInit.NbSectors = NbOfSectors;

	/* 擦除失败时停止写入，避免在未擦净扇区上继续编程造成数据不确定。 */
  if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
	{
    //擦除出错，返回
    return -2;
  }

  //以“字（4Byte）”的大小为单位写入数据
	TempAddr = StartWriteAddr;
  while (TempAddr < EndWriteAddr)
	{
		/* 当前 32 位字编程成功后才推进地址和数据索引，保证写入位置连续。 */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TempAddr, pBuffer[i]) == HAL_OK)
		{
			i++;
			TempAddr += 4;
    }
		else
	  {
			//写入出错，返回
      return -3;
    }
	}

  /* 给FLASH上锁，防止内容被篡改*/
  HAL_FLASH_Lock();

	return 0;
}


//======================================================================================================


//======================================================================================================

//======================================================================================================




//======================================================================================================
//======================================================================================================
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
  return *(__IO uint32_t*)faddr;
}

//======================================================================================================
//从指定地址开始读出指定长度的数据
//ReadAddr：起始地址
//pBuffer：数据指针
//NumToWrite：字(32位)数
void Flash_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint16_t NumToRead)
{
  uint16_t i = 0;
	uint32_t addr = ReadAddr;

  for (i = 0; i < NumToRead; i++)
  {
	  pBuffer[i] = STMFLASH_ReadWord(addr);  //读取4个字节.
	  addr += 4;  //偏移4个字节.
  }
}


















