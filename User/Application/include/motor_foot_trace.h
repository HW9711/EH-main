#ifndef MOTOR_FOOT_TRACE_H
#define MOTOR_FOOT_TRACE_H

#include <stdint.h>

/* 诊断事件只描述观测点，不直接改变脚踏、电机、泵或仲裁业务状态。 */
typedef enum
{
    MF_TRACE_BOOT = 1,
    MF_TRACE_FOOT_FRAME_VALID,
    MF_TRACE_FOOT_FRAME_BAD_OR_MISSING,
    MF_TRACE_FOOT_RUNTIME_TIMEOUT,
    MF_TRACE_FOOT_RELEASE_BOUNDARY,
    MF_TRACE_FOOT_STOP_LATCH_SET,
    MF_TRACE_FOOT_RUN_AUTHORIZED,
    MF_TRACE_FOOT_TASK_BLOCKED_BY_OWNER,
    MF_TRACE_OWNER_ENTER,
    MF_TRACE_OWNER_EXIT,
    MF_TRACE_OWNER_FORCE_RELEASE,
    MF_TRACE_MOTOR_TX_RUN,
    MF_TRACE_MOTOR_TX_STOP,
    MF_TRACE_MOTOR_TX_BLOCKED_BY_PARAM,
    MF_TRACE_DRIVER_FEEDBACK,
    MF_TRACE_FAULT_RELEASE_BUT_RUNFLAG,
    MF_TRACE_FAULT_RELEASE_BUT_RUN_TX,
    MF_TRACE_FAULT_STOP_TX_BUT_DRIVER_MOVING,
    MF_TRACE_FAULT_FOOT_TASK_BLOCKED_ON_RELEASE,
    MF_TRACE_MANUAL_FREEZE,
    MF_TRACE_CAPTURE_ARMED
} MotorFootTraceEvent_t;

/* 第一份异常触发后冻结环形缓冲，后续异常不能覆盖第一次现场。 */
typedef enum
{
    MF_TRIGGER_NONE = 0,
    MF_TRIGGER_RELEASE_BUT_RUNFLAG,
    MF_TRIGGER_RELEASE_BUT_RUN_TX,
    MF_TRIGGER_STOP_TX_BUT_DRIVER_MOVING,
    MF_TRIGGER_FOOT_TASK_BLOCKED_ON_RELEASE,
    MF_TRIGGER_MANUAL
} MotorFootTraceTrigger_t;

/* 每条记录同时保存脚踏、仲裁、业务请求、最终 UART1 命令和驱动反馈快照。 */
typedef struct
{
    uint32_t tick_ms;                    /* 主控 HAL 毫秒时钟。 */
    uint16_t record_sequence;            /* 环形记录序号，自然回绕。 */
    uint16_t foot_frame_sequence;        /* CRC 正确的 UART4 实时帧序号。 */
    uint8_t event;                       /* MotorFootTraceEvent_t。 */
    uint8_t trigger;                     /* MotorFootTraceTrigger_t。 */
    uint8_t pedal_type;                  /* 1 单踏板、2 双段、3 双脚踏。 */
    uint8_t foot_runtime_valid;          /* 当前脚踏实时帧门禁。 */
    uint16_t adc_left;                   /* 单踏板/双段使用左值，双脚踏为左侧值。 */
    uint16_t adc_right;                  /* 双脚踏右侧值，其它类型为 0。 */
    uint16_t low_left;                   /* 左侧低点定标。 */
    uint16_t mid_left;                   /* 左侧中点定标。 */
    uint16_t high_left;                  /* 左侧高点定标。 */
    uint16_t low_right;                  /* 右侧低点定标。 */
    uint16_t mid_right;                  /* 右侧中点定标。 */
    uint16_t high_right;                 /* 右侧高点定标。 */
    uint8_t foot_stop_latched;           /* 脚踏停机锁存。 */
    uint8_t foot_release_ready;          /* 已观察到真实释放，可等待下一次踩下。 */
    uint8_t foot_active_source;          /* 0 无、1 左、2 右。 */
    uint8_t control_owner;               /* CONTROL_OWNER_*。 */
    uint8_t drive_type;                  /* WorkMessage.drivetype_work。 */
    uint8_t runflag_work;                /* 业务运行请求。 */
    uint8_t jt_left_flag;                /* 左脚踏电机控制标志。 */
    uint8_t jt_right_flag;               /* 右脚踏电机控制标志。 */
    uint32_t speed_work;                 /* 脚踏实时目标速度，单位 rpm。 */
    uint16_t motor_command_sequence;     /* 最终 UART1 命令变化序号。 */
    uint8_t motor_command_run;           /* 最终命令 1=RUN、0=STOP。 */
    uint8_t motor_command_type;          /* 最终驱动电机类型。 */
    uint32_t motor_command_rpm;           /* 最终 UART1 命令速度，单位 rpm。 */
    uint16_t motor_feedback_sequence;    /* CRC 正确的驱动反馈序号。 */
    uint8_t motor_feedback_valid;        /* 驱动反馈快照是否存在。 */
    uint8_t motor_feedback_error;        /* 驱动原始 Err。 */
    uint32_t motor_feedback_rpm;         /* 驱动反馈速度，单位 rpm。 */
    uint16_t extra0;                     /* 事件附加值 0，按事件解释。 */
    uint16_t extra1;                     /* 事件附加值 1，按事件解释。 */
} MotorFootTraceRecord_t;

/* 以下变量只供临时诊断固件在调试器中观察，不参与手柄、脚踏、泵或电机控制判定。 */
extern volatile int32_t g_motor_foot_trace_task_create_result;    /* 1 表示诊断任务创建成功，0 表示失败。 */
extern volatile int32_t g_motor_foot_trace_task_start_result;     /* 1 表示 30ms 周期任务启动成功，0 表示失败。 */
extern volatile uint32_t g_motor_foot_trace_task_run_count;       /* 诊断任务实际进入次数，持续增加可证明调度正常。 */
extern volatile uint32_t g_motor_foot_trace_alive_attempt_count;  /* 固定在线帧发送尝试次数。 */
extern volatile uint32_t g_motor_foot_trace_alive_success_count;  /* HAL 返回 HAL_OK 的在线帧次数。 */
extern volatile uint32_t g_motor_foot_trace_alive_last_status;    /* 最近一次 HAL 状态：0=OK、1=ERROR、2=BUSY、3=TIMEOUT。 */
extern volatile uint32_t g_motor_foot_trace_alive_last_tick;      /* 最近一次尝试发送在线帧的主控毫秒时刻。 */

/* 函数功能：初始化诊断环形缓冲、正常在线帧和故障后 UART2 导出任务。输入参数：无。返回参数：无。 */
void MotorFootTrace_Init(void);

/* 函数功能：记录一个诊断事件及两个附加值。输入参数：event、extra0、extra1。返回参数：无。 */
void MotorFootTrace_Record(MotorFootTraceEvent_t event, uint16_t extra0, uint16_t extra1);

/* 函数功能：按真实 UART4 实时帧更新采样节流并记录关键 ADC 变化。输入参数：脚踏类型和左右 ADC。返回参数：无。 */
void MotorFootTrace_OnValidFootFrame(uint8_t pedal_type, uint16_t adc_left, uint16_t adc_right);

/* 函数功能：记录脚踏进入电机释放区并开始释放保护窗口。输入参数：无。返回参数：无。 */
void MotorFootTrace_OnFootRelease(void);

/* 函数功能：新踩踏通过先松后踩门禁时结束旧释放窗口。输入参数：无。返回参数：无。 */
void MotorFootTrace_OnFootRunAuthorized(void);

/* 函数功能：对脚踏任务被其它控制源阻塞的周期做去重记录。输入参数：owner、runflag。返回参数：无。 */
void MotorFootTrace_OnFootOwnerBlocked(uint8_t owner, uint8_t runflag);

/* 函数功能：记录最终送入 UART1 的 RUN/STOP 命令并检查释放后误发 RUN。输入参数：最终命令字段。返回参数：无。 */
void MotorFootTrace_OnMotorCommand(uint8_t run_state,
                                   uint16_t command_sequence,
                                   uint32_t command_rpm,
                                   uint8_t motor_type);

/* 函数功能：记录 CRC 正确的驱动反馈并检查 STOP 后仍明显转动。输入参数：反馈快照字段。返回参数：无。 */
void MotorFootTrace_OnDriverFeedback(uint8_t feedback_valid,
                                     uint16_t feedback_sequence,
                                     uint32_t speed_rpm,
                                     uint8_t raw_error,
                                     uint16_t current_x100);

/* 函数功能：自动条件只记录一次疑似事件，人工触发才冻结现场并准备延时导出。输入参数：trigger。返回参数：无。 */
void MotorFootTrace_Trigger(MotorFootTraceTrigger_t trigger);

/* 函数功能：人工冻结当前现场。输入参数：无。返回参数：无。 */
void MotorFootTrace_ForceManualFreeze(void);

#endif /* MOTOR_FOOT_TRACE_H */
