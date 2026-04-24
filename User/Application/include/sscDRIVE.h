#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define JMB_CURRENT 330 //减速比尾号
#define TMBB_CURRENT 330//其他的手柄类型
#define MX_YIM_CURRENT_X2 350//其他的手柄类型
#define MX_YIM16_CURRENT_x2783 450//其他的手柄类型
#define MX_YIP_CURRENT 260//其他的手柄类型
#define TMBA_CURRENT 450//其他的手柄类型
#define EMBA_CURRENT 450//其他的手柄类型
#define EMBB_CURRENT 450//其他的手柄类型
#define PXBA_CURRENT_496 120//其他的手柄类型
#define PXBB_CURRENT_496 120//其他的手柄类型
#define PXBA_CURRENT 220//其他的手柄类型
#define PXBB_CURRENT 220//其他的手柄类型
#define PXBA_CURRENT_X2 360//其他的手柄类型
#define PXBB_CURRENT_X2 360//其他的手柄类型
#define PXBA_CURRENT_196 280//其他的手柄类型
#define PXBB_CURRENT_196 280//其他的手柄类型
#define PX_YIM_CURRENT_2 66
#define PX_YIP_CURRENT_45 40

void SscDriveMotorTask_Init(void);
void ToolPosMay(uint8_t channel_number,bool direction,uint8_t angel);//通道，方向，角度
