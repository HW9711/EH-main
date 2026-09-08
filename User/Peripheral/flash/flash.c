
#include "stm32f4xx_hal.h"
#include "flash.h"

//本接口允许访问的Flash容量，按硬件和固件分区核对，不能当作扩大存储空间的开关。
#define FLASH_SIZE   1024 	 		//单位KB，用于写入地址检查；当前限制为从Flash基地址起的1MB。

//FLASH起始地址
#define STM32_FLASH_BASE    0x08000000 	//STM32 FLASH的起始地址

//======================================================================================================
/*
 * 函数功能：根据地址查找所在Flash扇区；本函数不检查地址是否有效。
 * 输入参数：address为Flash字节地址，调用方须先检查范围。
 * 返回参数：FLASH_SECTOR_0至FLASH_SECTOR_11中的扇区编号。
 */
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
/*
 * 函数功能：擦除从WriteAddr到WriteAddr+NumToWrite所在的全部扇区（含末地址所在扇区），再每次写4字节。
 * 输入参数：WriteAddr为起始字节地址，pBuffer为32位数据数组，NumToWrite为字节数，应为4的倍数。
 * 返回参数：0成功，-1地址超范围，-2擦除失败，-3写入失败；末地址刚好进入下一扇区时也会擦掉该扇区，长度为0仍会擦除。
 */
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

  //包括EndWriteAddr所在扇区，即使结束地址刚好等于下一扇区起点；函数不检查其中是否有其它需保留数据。
  //Fill EraseInit structure
	FlashEraseInit.Banks = FLASH_BANK_1;  //操作的扇区块
  FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; //按扇区擦除，不能只擦除本次写入的几个字节。
  FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; //选择HAL规定的供电电压档位；写入宽度由下方FLASH_TYPEPROGRAM_WORD指定。
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

  /* 写入成功后关闭Flash写操作，防止程序误写；上方错误返回不会经过这里。 */
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
/*
 * 函数功能：从Flash连续读取32位数据，不做地址范围检查。
 * 输入参数：ReadAddr为起始字节地址，pBuffer为输出数组，NumToRead为32位数据个数（不是字节数）。
 * 返回参数：无。
 */
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


















