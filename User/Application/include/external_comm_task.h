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

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_TASK_H */
