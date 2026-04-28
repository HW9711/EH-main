#ifndef __EXTERNAL_COMM_TASK_H
#define __EXTERNAL_COMM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 UART2 外部通信任务：独占 UART2 接收解析，并按周期上传心跳。 */
void ExternalComm_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_TASK_H */
