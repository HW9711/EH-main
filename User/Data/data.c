//data.c

#include "data.h"

ModelConfig SysModelConfig = { 0 };

HandleData SysHandleData = { 0 };

InterfaceData SysInterface =
{
  {0, 0, 0, 0, 0}, {1, 1, 1, 1, 1}, 0xff,
   0, 0, 0, 0, 0, 0xff, 0
};

FootPedalData SysFootPedalData = { 0 };

UIDisplayData SysUIDisplayData = { 0 };

SystemRunParam SysRunData =
{
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 0, 0, 0,
  0, 0, 0, 0, 0, 1, 0,
  {0, 0}, 0, 0, 0, 0, 0, 0, 0,
  {0, 0}, {0, 0, 0, 0, 0}, {0, 0},
  {0x24, 0x04, 0x16, 0x13, 0x45, 0x20},
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

SystemSetParam SysSetParam[5] =
{
  {
	  0, 0, 0, 0, 0, 0, 0,
	  12, 40, 1, 0, 0, 1, 1,
	  0, 1, 3, 4, 1,12, 0
  },
  {
	  0, 0, 0, 0, 0, 0, 0,
	  12, 40, 1, 0, 0, 1, 1,
	  0, 1, 3, 4, 1,12, 0
  },
  {
	  0, 0, 0, 0, 0, 0, 0,
	  12, 40, 1, 0, 0, 1, 1,
	  0, 1, 3, 4, 1,12, 0
  },
  {
	  0, 0, 0, 0, 0, 0, 0,
	  12, 40, 1, 0, 0, 1, 1,
	  0, 1, 3, 4, 1,12, 0
  },
  {
	  0, 0, 0, 0, 0, 0, 0,
	  12, 40, 1, 0, 0, 1, 1,
	  0, 1, 3, 4, 1,12, 0
  },
};

//============================================================================
// 函数名称: Data_OnLineCnt_Get()
// 功能描述: 返回当前在线手柄数量
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
uint8_t Data_GetOnLineCnt(void)
{
  uint8_t n = 0;

  if (SysInterface.Interface[0] > 0)
	  n++;

  if (SysInterface.Interface[1] > 0)
	  n++;

  if (SysInterface.Interface[2] > 0)
	  n++;

  if (SysInterface.Interface[3] > 0)
	  n++;

  if (SysInterface.Interface[4] > 0)
	  n++;

  return n;
}






