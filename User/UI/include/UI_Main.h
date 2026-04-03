//UI_Main.h

#ifndef __UI_MAIN_H
#define __UI_MAIN_H

#include <stdint.h>

//显示的接口连接状态，显示的在线手柄数量
typedef struct UIInterfaceTag
{
  uint8_t UIInterfaceS[5];   //显示的接口类型，根据数字的大小判断被选中状态 0离线  【下标：0 --1号手柄，1 --2号手柄，2 --3号手柄，3 --4号手柄，4 --5号手柄】

  uint8_t UIInterfaceCnt;  //显示的在线手柄数量

  uint8_t State_diam;   //直径 0
  uint8_t State_length; //长度 1
  uint8_t State_angle;  //角度 2

  uint8_t State_Info1; //信息状态转速 3
  uint8_t State_Info2; //信息状态流量 4
  uint8_t State_Info3; //信息状态频率 5
 
  uint8_t State_Cutter1;      //刀具图片 6
  uint8_t State_Cutter2;      //刀具信息 7

  //0不显示脚控/手控,
  //1脚控未连接 未选中,
  //2脚控 已选中 选中,
  //3手控 已选中 选中,
  //4脚控已连接\手控已选中,
  //5脚控已选中\手控已连接
  uint8_t State_FootPedal;    //脚控 状态 8
  uint8_t State_Info4; //信息状态频率 9
//	u8 SelectWindowNum;
//	u8 SelectWindowNumLast;
//	u16 WindowDisappearTimeCnt;

} UIInterface;

//============================================================================
// 函数名称: UIMain_RefreshTaskInit()
// 功能描述: 主UI 刷新任务
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
void UIMain_RefreshTaskInit(void);

#endif  //__UI_MAIN_H


