#ifndef __EXTERNAL_COMM_TASK_H
#define __EXTERNAL_COMM_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一次外控连接只能使用正式或简易协议中的一种；登录帧不能混着计数，另一协议也不能重置断线计时。 */
typedef enum
{
    EXTERNAL_COMM_PROTOCOL_SOURCE_NONE = 0U,   /* 当前没有完成三帧确认的外部通信会话。 */
    EXTERNAL_COMM_PROTOCOL_SOURCE_FORMAL,      /* 当前会话由带CRC的正式外部通信协议建立。 */
    EXTERNAL_COMM_PROTOCOL_SOURCE_SIMPLE       /* 当前会话由AA BB CC固定帧简易协议建立。 */
} ExternalCommProtocolSource_t;

/* 初始化 UART2 外部通信任务：独占 UART2 接收解析，并按周期上传心跳。 */
void ExternalComm_Init(void);
/* 临时上传一个报警码，保持 hold_ms 后通信任务自动上传 0，供运行中另一路手柄校验失败弹窗限时显示。 */
void ExternalComm_SendTransientAlarm(uint8_t alarm_value, uint16_t hold_ms);
/* 手柄运行中掉线时清除外控层保存的注水泵跟随请求，避免后续外控刷新重新拉起联动泵。 */
void ExternalComm_ClearHandleInjectionPumpFollow(void);
/* 泵因压力或驱动保护停机时，清除外控保存的运行请求，防止后续刷新再次启动故障泵。 */
void ExternalComm_ClearPumpRunRequest(uint8_t pump_channel);
/* 屏幕确认退出外控后提交退出请求，由外部通信任务统一释放控制权并通知上位机。 */
void ExternalComm_RequestExit(void);

/*
 * 函数功能：收到当前协议的有效命令后，重置断线计时并刷新小电脑图标；不自动恢复已停止的电机和泵。
 * 输入参数：无。
 * 返回参数：无。
 */
void ExternalComm_NotifyLink(void);

/*
 * 函数功能：首次登录时，要求同一种协议在 300ms 内收到三份内容一致的申请，收齐后才允许申请控制权。
 * 输入参数：source 为正式或简易协议；identity 为要比较的授权数据；identity_len 为字节数，正式协议为 8，简易协议为 0。
 * 返回参数：本协议已完成三帧确认返回 1；未收齐、另一协议已连接或参数错误返回 0。
 */
uint8_t ExternalComm_TryConfirmProtocol(ExternalCommProtocolSource_t source,
                                        const uint8_t *identity,
                                        uint16_t identity_len);

/*
 * 函数功能：把内部命令交给原外控处理函数执行，但不向 UART2 发送正式协议应答。
 * 输入参数：fun_code、area_code 为原协议命令字段；info_area 为可选命令数据，info_len 为其字节数。
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
