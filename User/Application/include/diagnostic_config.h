#ifndef DIAGNOSTIC_CONFIG_H
#define DIAGNOSTIC_CONFIG_H

/* 临时诊断固件开关：1 表示启用脚踏/手柄异常转动追踪，测试结束后必须改回 0。 */
#define MOTOR_FOOT_TRACE_ENABLE                  1U

/* 诊断期间 UART2 只输出 EHDBG 文本，原外控协议、心跳、应答和遥测全部停用。 */
#define MOTOR_FOOT_TRACE_UART2_EXCLUSIVE         1U

/* 诊断串口使用 115200 8N1，避免单条 CSV 在 30ms 导出周期内发送不完。 */
#define MOTOR_FOOT_TRACE_UART2_BAUD              115200U

/* 正常诊断期每 2 秒只发送一次固定在线帧，供上位机确认主控确实在线。 */
#define MOTOR_FOOT_TRACE_ALIVE_PERIOD_MS          2000U

/* 人工确认异常后才冻结；自动条件只写一条疑似事件，不能提前占用本轮唯一故障现场。 */
#define MOTOR_FOOT_TRACE_MANUAL_FREEZE_ONLY         1U

/* 固定 512 条 RAM 环形记录，不使用动态内存，为人工发现异常并点击冻结保留足够反应窗口。 */
#define MOTOR_FOOT_TRACE_CAPACITY                512U

/* 人工冻结 300ms 后开始导出，每 30ms 最多发送一行，避免长时间连续阻塞。 */
#define MOTOR_FOOT_TRACE_DUMP_PERIOD_MS          30U
#define MOTOR_FOOT_TRACE_DUMP_DELAY_MS           300U

/* STOP 后超过 300ms 仍反馈大于 300rpm 时只记录疑似事件，不自动冻结也不参与正式停机判断。 */
#define MOTOR_FOOT_TRACE_DRIVER_MOVING_RPM       300U
#define MOTOR_FOOT_TRACE_STOP_FEEDBACK_GRACE_MS  300U

/* 观察到脚踏释放后 1500ms 内若仍发送脚踏 RUN，则记录疑似事件供人工冻结后复核。 */
#define MOTOR_FOOT_TRACE_RELEASE_GUARD_MS         1500U

#endif /* DIAGNOSTIC_CONFIG_H */
