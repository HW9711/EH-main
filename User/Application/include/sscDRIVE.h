#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* 以下是旧版按型号设置的保护电流，当前工程没有引用；修改它们不会改变实际保护电流。
 * 当前发送值来自 WorkMessage.current_work，单位 0.01A；手柄参数通常由 EEPROM Page4[21..22] 提供。
 * 型号后缀仅保留旧命名，不能据此修改当前手柄的减速比或电流设置。
 */
#define JMB_CURRENT 330 // JMB 旧保护电流：330 对应 3.30A，当前未使用。
#define TMBB_CURRENT 330// TMBB 旧保护电流：3.30A，当前未使用。
#define MX_YIM_CURRENT_X2 350// MX_YIM 的 X2 旧配置：3.50A，当前未使用。
#define MX_YIM16_CURRENT_x2783 450// MX_YIM16 的 x2783 旧配置：4.50A，当前未使用。
#define MX_YIP_CURRENT 260// MX_YIP 旧保护电流：2.60A，当前未使用。
#define TMBA_CURRENT 450// TMBA 旧保护电流：4.50A，当前未使用。
#define EMBA_CURRENT 450// EMBA 旧保护电流：4.50A，当前未使用。
#define EMBB_CURRENT 450// EMBB 旧保护电流：4.50A，当前未使用。
#define PXBA_CURRENT_496 120// PXBA 的 496 旧配置：1.20A，当前未使用。
#define PXBB_CURRENT_496 120// PXBB 的 496 旧配置：1.20A，当前未使用。
#define PXBA_CURRENT 220// PXBA 旧保护电流：2.20A，当前未使用。
#define PXBB_CURRENT 220// PXBB 旧保护电流：2.20A，当前未使用。
#define PXBA_CURRENT_X2 360// PXBA 的 X2 旧配置：3.60A，当前未使用。
#define PXBB_CURRENT_X2 360// PXBB 的 X2 旧配置：3.60A，当前未使用。
#define PXBA_CURRENT_196 280// PXBA 的 196 旧配置：2.80A，当前未使用。
#define PXBB_CURRENT_196 280// PXBB 的 196 旧配置：2.80A，当前未使用。
#define PX_YIM_CURRENT_2 66 // PX_YIM 的 2 旧配置：0.66A，当前未使用。
#define PX_YIP_CURRENT_45 40 // PX_YIP 的 45 旧配置：0.40A，当前未使用。

/* 保存最近一次 50ms 任务的启停判断，以及最近一次内容变化的 UART1 命令，供状态查询和调试。 */
typedef struct
{
    uint32_t evaluated_tick_ms; /* HAL毫秒时钟：最近一次50ms输出任务完成启停判定的时刻。 */
    uint32_t sent_tick_ms;      /* HAL 毫秒时钟：不同命令实际送入 UART1 的时刻。 */
    uint32_t source_speed_rpm;  /* 当前控制来源提供的原始目标速度，尚未做机械倍率和低速补偿。 */
    uint32_t command_speed_rpm; /* 驱动帧 byte4~5 换算后的电机指令速度，停机固定为 0rpm。 */
    uint16_t sequence;          /* 命令内容改变时加 1，超过 65535 从 0 开始；每 50ms 重发相同帧时不加。 */
    uint8_t channel;            /* 命令形成时的逻辑 A/B 通道。 */
    uint8_t run_state;          /* 1表示启动帧，0表示停止帧。 */
    uint8_t direction;          /* 实际 UART1 帧 byte1 控制模式。 */
    uint8_t motor_type;         /* 实际 UART1 帧 byte3 驱动电机类型。 */
    uint8_t requested_run_state;/* WorkMessage在本次50ms判定时提出的原始运行请求。 */
    uint8_t drive_type;         /* 本次请求来源：脚踏、手控、触控或外控，数值沿用drivetype_work。 */
    uint8_t zero_speed_blocked; /* 1 表示虽然要求启动，但原始速度或除以 10 后的协议速度为 0，实际改发停止帧。 */
    uint8_t valid;              /* 已形成至少一份周期电机命令时置1。 */
} MotorDriveCommandSnapshot_t;

void SscDriveMotorTask_Init(void);
void ToolPosMay(uint8_t channel_number,bool direction,uint8_t angel);//通道，方向，角度

/*
 * 函数功能：读取电机请求和已发送命令的记录，读取途中发生更新时重试，避免混用新旧数据。
 * 输入参数：snapshot指向接收命令记录的结构体，复制后调用方读取自己的这份数据。
 * 返回参数：已有有效命令记录且复制期间未被修改时返回1，否则返回0。
 */
uint8_t MotorDrive_CopyCommandSnapshot(MotorDriveCommandSnapshot_t *snapshot);
