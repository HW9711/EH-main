//encryption.c

#include "stm32f4xx_hal.h"
#include "encryption.h"
#include "common.h"
#include "delay.h"
#include "eeprom.h"
#include "data.h"
#include <stdint.h>

/*
UID 自定算法加密
全球唯一 ID 自定义算法加密配置:
由于STM8/STM32有12个字节的全球唯一ID,我们把这12个字节的唯一ID按地址从低到高
分别记为:
ID[0],ID[1],ID[2],ID[3],ID[4],ID[5],ID[6],ID[7],ID[8],ID[9],ID[10],ID[11]
在加密公式中,输入参数有两个：
1. uint8_t D[12];//D[12]为公式的输入数组(注意与ID[12]区分).
2. uint32_t Fml_Constant; // 用户指定的32位常数
加密公式的输出为一个数组:
uint8_t Result[4]; // 公式计算结果输出,四个字节
软件上的＂存放起始地址＂的意义就是指这个算出的结果的存储地址．存储字节数是
指这个结果的前几个字节保存到芯片的存储器中．
关于输入参数D:
如图(在软件中可配置D数组的每个元素所赋的ID的值)
如上图红框中D[0]后的下拉框中值是4,意即把ID[4]赋值给D[0],依次类推,
即:
D[0]=ID[4];
D[1]=ID[8];
D[2]=ID[1];
D[3]=ID[3];
D[4]=ID[0];
D[5]=ID[5];
D[6]=ID[10];(注意,软件中用十六进制的A表示了)
D[7]=ID[7];
D[8]=ID[9];
D[9]=ID[2];
D[10]=ID[11]; (注意,软件中用十六进制的B表示了)
D[11]=ID[6];
这样，就算别人拿到了公式或使用同一个公式，也会产生不同的结果。因为输入的
组合有2^32*12^12个.
关于输入参数Fml_Constant:
Fml_Constant的值可以软件中配置:
这个常数为一32位常量.软件中以十六进制配置.
用户常见问题解答:
1. 在自已的软件中如何读芯片的唯一ID呢?
非常简单,因为STM8,STM32为线性编址.读取数据无须专门处理.只要把相应的地
址转换成指针类型就可以了.
比如要读芯片的全球唯一ID值:
uint8_t* UID=(uint8_t*)ID的起始地址;
以后只要当数组用就可以了.比如UID[0]就是芯片最低字节的全球唯一ID.
全球唯一 ID 自定义算法加密公式(同时适用于 STM8,STM32 脱机编程器):
// 请注意，因为存在不同的编译器，运算出来的结果可能会与编程器不对应该时，请把大于
8 位的数据进行高低交换后再进行运算.注意对于 STM8,所有编译器均默认为大端模式.要进
行高低字节转换.对于 STM32,GD32 则一般默认就是小端模式,无须特殊处理.
*/


/*
* 函数名：Get_ChipID
 * 描述  ：获取芯片ID
 * 输入  ：无
 * 输出  ：无
 * 说明  ：96位的ID是stm32唯一身份标识，可以以8bit、16bit、32bit读取
           提供了大端和小端两种表示方法
					 */
void Encryption_Check(void)
{
//    u32 ChipUniqueID[3];
//     地址从小到大,先放低字节，再放高字节：小端模式
//     地址从小到大,先放高字节，再放低字节：大端模式
// ChipUniqueID[2] = *(__IO u32*)(0X1FFFF7E8);  // 低字节
// ChipUniqueID[1] = *(__IO u32 *)(0X1FFFF7EC); //
// ChipUniqueID[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节

//////对应的存放起始地址为：#define  FLASH_WriteAddress     0x0807F800
  uint32_t Fml_Constant = 0x12f333b;   // 输入到公式的常数  0x12f333b  19870523
  uint8_t *C = (uint8_t*)&Fml_Constant;  //把 公式的常数 转换为数组

  uint8_t temp[12] = { 0 }, uidTemp[12] = { 0 };
  uint32_t UID_L = 0, UID_M = 0, UID_H = 0;

  uint8_t ResultID[4] = { 0 };    //通过运算后产生的数组
  uint8_t UIDEepromBuff[4] = { 0 };

  UID_L = *(__IO uint32_t*)(0x1FFFF7E8);    //产品唯一身份标识寄存器（96位）// 低字节
  UID_M = *(__IO uint32_t*)(0x1FFFF7EC);
  UID_H = *(__IO uint32_t*)(0x1FFFF7F0);    // 高字节

  temp[0]  = (uint8_t)(UID_L & 0x000000FF);
  temp[1]  = (uint8_t)((UID_L & 0x0000FF00) >> 8);
  temp[2]  = (uint8_t)((UID_L & 0x00FF0000) >> 16);
  temp[3]  = (uint8_t)((UID_L & 0xFF000000) >> 24);

  temp[4]  = (uint8_t)(UID_M & 0x000000FF);
  temp[5]  = (uint8_t)((UID_M & 0x0000FF00) >> 8);
  temp[6]  = (uint8_t)((UID_M & 0x00FF0000) >> 16);
  temp[7]  = (uint8_t)((UID_M & 0xFF000000) >> 24);

  temp[8]  = (uint8_t)( UID_H & 0x000000FF);
  temp[9]  = (uint8_t)((UID_H & 0x0000FF00) >> 8);
  temp[10] = (uint8_t)((UID_H & 0x00FF0000) >> 16);
  temp[11] = (uint8_t)((UID_H & 0xFF000000) >> 24);

  ///////对应编程数组赋值
  uidTemp[0] = temp[0];
  uidTemp[1] = temp[1];
  uidTemp[2] = temp[2];

  uidTemp[3] = temp[8];

  uidTemp[4] = temp[4];

  uidTemp[5] = temp[3];

  uidTemp[6] = temp[6];
  uidTemp[7] = temp[7];
  uidTemp[8] = temp[8];
  uidTemp[9] = temp[9];

  uidTemp[10]= temp[3];

  uidTemp[11]= temp[11];

  ResultID[0] = C[0] + uidTemp[0] ^ uidTemp[9] - uidTemp[4] ;
  ResultID[1] = C[1] - uidTemp[8] - uidTemp[2] ^ uidTemp[5] ;
  ResultID[2] = C[2] ^ uidTemp[6] - uidTemp[11] - uidTemp[3] ;
  ResultID[3] = (C[3] & uidTemp[1]) ^ uidTemp[10] + uidTemp[7] ;

  EEPROM_AT24CXX_Read(0x00, UIDEepromBuff, 4);

  /*
  UID_Read_Buff[0] = AT24CXX_ReadOneByte(0X00);
  Delay_ms(5);

  UID_Read_Buff[1] = AT24CXX_ReadOneByte(0X01);
  Delay_ms(5);

  UID_Read_Buff[2] = AT24CXX_ReadOneByte(0X02);
  Delay_ms(5);

  UID_Read_Buff[3] = AT24CXX_ReadOneByte(0X03);
  Delay_ms(5);
  */

  if (Common_CompareData(ResultID, UIDEepromBuff, 4))
    SysRunData.EncryptionCheckFlag = No_Error;
  else
    SysRunData.EncryptionCheckFlag = Error;
}








