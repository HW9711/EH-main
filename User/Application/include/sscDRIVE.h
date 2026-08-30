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

/* 电机命令快照同时保存最近一次50ms输出判定和最近一次实际改变并送入UART1的驱动命令。 */
typedef struct
{
    uint32_t evaluated_tick_ms; /* HAL毫秒时钟：最近一次50ms输出任务完成启停判定的时刻。 */
    uint32_t sent_tick_ms;      /* HAL 毫秒时钟：不同命令实际送入 UART1 的时刻。 */
    uint32_t source_speed_rpm;  /* 当前控制来源提供的原始目标速度，尚未做机械倍率和低速补偿。 */
    uint32_t command_speed_rpm; /* 驱动帧 byte4~5 换算后的电机指令速度，停机固定为 0rpm。 */
    uint16_t sequence;          /* 不同命令序号，自然回绕；重复的50ms保活帧不递增。 */
    uint8_t channel;            /* 命令形成时的逻辑 A/B 通道。 */
    uint8_t run_state;          /* 1表示启动帧，0表示停止帧。 */
    uint8_t direction;          /* 实际 UART1 帧 byte1 控制模式。 */
    uint8_t motor_type;         /* 实际 UART1 帧 byte3 驱动电机类型。 */
    uint8_t requested_run_state;/* WorkMessage在本次50ms判定时提出的原始运行请求。 */
    uint8_t drive_type;         /* 本次请求来源：脚踏、手控、触控或外控，数值沿用drivetype_work。 */
    uint8_t zero_speed_blocked; /* 1表示请求RUN但原始或协议量化速度为0，本周期实际发送STOP。 */
    uint8_t valid;              /* 已形成至少一份周期电机命令时置1。 */
} MotorDriveCommandSnapshot_t;

void SscDriveMotorTask_Init(void);
void ToolPosMay(uint8_t channel_number,bool direction,uint8_t angel);//通道，方向，角度

/*
 * 函数功能：复制最近一次实际改变并送入 UART1 的电机命令一致性快照。
 * 输入参数：snapshot 指向调用方提供的快照缓存。
 * 返回参数：快照有效且复制成功返回1，否则返回0。
 */
uint8_t MotorDrive_CopyCommandSnapshot(MotorDriveCommandSnapshot_t *snapshot);
