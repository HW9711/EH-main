#ifndef __EXTERNAL_COMM_TASK_H
#define __EXTERNAL_COMM_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 UART2 外部通信任务：独占 UART2 接收解析，并按周期上传心跳。 */
void ExternalComm_Init(void);
/* 临时上传一个报警码，保持 hold_ms 后通信任务自动上传 0，供运行中另一路手柄校验失败弹窗限时显示。 */
void ExternalComm_SendTransientAlarm(uint8_t alarm_value, uint16_t hold_ms);
/* 手柄运行中掉线时清除外控层保存的注水泵跟随请求，避免后续外控刷新重新拉起联动泵。 */
void ExternalComm_ClearHandleInjectionPumpFollow(void);
/* 压力保护停泵时清除外控层对应泵的运行请求，避免旧外控锁存再次拉起已堵塞的泵。 */
void ExternalComm_ClearPumpPressureRunRequest(uint8_t pump_channel);
/* 屏幕确认退出外控后提交退出请求，由外部通信任务统一释放控制权并通知上位机。 */
void ExternalComm_RequestExit(void);

/*
 * 函数功能：刷新外控链路活动时间，供同一 UART2 上的协议适配层复用原安全超时。
 * 输入参数：无。
 * 返回参数：无。
 */
void ExternalComm_NotifyLink(void);

/*
 * 函数功能：静默复用原外控协议业务分发，不向 UART2 发送原协议 ACK。
 * 输入参数：fun_code、area_code 为原协议分发字段；info_area 和 info_len 为可选载荷。
 * 返回参数：无。
 */
void ExternalComm_RunSilent(uint8_t fun_code,
                            uint8_t area_code,
                            const uint8_t *info_area,
                            uint16_t info_len);

/*
 * 函数功能：查询 A/B 泵是否存在外控独立运行请求。
 * 输入参数：pump_channel 为 CHANNEL_A 或 CHANNEL_B。
 * 返回参数：存在请求返回 1，否则返回 0。
 */
uint8_t ExternalComm_PumpRunRequested(uint8_t pump_channel);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_TASK_H */
