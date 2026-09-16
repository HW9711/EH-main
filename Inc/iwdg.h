/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   This file contains all the function prototypes for
  *          the iwdg.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_IWDG_Init(void);

/* USER CODE BEGIN Prototypes */
/* 业务检查间隔，单位ms；调大后发现任务停止执行的时间也会延后。 */
#define IWDG_CHECK_PERIOD_MS 100U
/* 关键任务两次完成之间允许的最大间隔，单位ms；包含等待公共锁的时间。 */
#define IWDG_TASK_TIMEOUT_MS 1000U
/* 调度器启动后等待所有关键任务首次完成的上限，单位ms；覆盖首个线程的1500ms调试启动等待。 */
#define IWDG_TASK_START_TIMEOUT_MS 3000U

typedef enum
{
    IWDG_STAGE_CLOCK = 0U,       /* 已启用硬件看门狗，正在配置系统时钟。 */
    IWDG_STAGE_PERIPHERALS,     /* 正在初始化串口、定时器和I2C外设。 */
    IWDG_STAGE_EEPROM,          /* 正在初始化EEPROM并同步主控版本。 */
    IWDG_STAGE_OUTPUT_STOP,     /* 正在准备业务串口并发送开机停止命令。 */
    IWDG_STAGE_STARTUP_PAGE,    /* 启动页入口窗口，可在规定总时间内循环检查。 */
    IWDG_STAGE_SETTLE,          /* 启动页结束后的固定等待，不能无限续期。 */
    IWDG_STAGE_RFID,            /* 正在初始化射频模块。 */
    IWDG_STAGE_TASKS,           /* 正在创建业务任务和公共锁。 */
    IWDG_STAGE_WAIT_TASKS,      /* 调度器开始运行，等待所有关键任务完成第一轮。 */
    IWDG_STAGE_RUNNING,         /* 所有关键任务已完成过，按最近完成时间检查。 */
    IWDG_STAGE_CALIBRATION,     /* 独立脚踏定标循环，正常业务任务尚未创建。 */
    IWDG_STAGE_FAULT            /* 已锁定程序故障，任何入口都不能继续喂狗。 */
} IwdgStage_t;

typedef enum
{
    IWDG_FAULT_NONE = 0U,       /* 没有软件保存的故障，硬件复位原因仍看reset_flags。 */
    IWDG_FAULT_BOOT_TIMEOUT,    /* 开机步骤未在规定时间内完成。 */
    IWDG_FAULT_TASK_MISSING,    /* 必须运行的任务没有创建成功或被停止。 */
    IWDG_FAULT_TASK_TIMEOUT,    /* 关键任务超过规定时间没有完成一轮。 */
    IWDG_FAULT_STAGE,           /* 启动或喂狗调用顺序错误。 */
    IWDG_FAULT_HARD,            /* 处理器进入HardFault。 */
    IWDG_FAULT_MEMORY,          /* 处理器进入MemManage异常。 */
    IWDG_FAULT_BUS,             /* 处理器进入BusFault异常。 */
    IWDG_FAULT_USAGE,           /* 处理器进入UsageFault异常。 */
    IWDG_FAULT_INIT             /* 外设或看门狗任务初始化失败。 */
} IwdgFault_t;

typedef struct
{
    uint32_t reset_flags;       /* 本次启动时读取的RCC_CSR，读取后清除硬件复位标志。 */
    uint32_t fault_reason;      /* 上次软件记录的IwdgFault_t；硬件直接复位时可能为0。 */
    uint32_t overdue_task;      /* 超时或缺失的任务编号，1起对应iwdg.c的任务表，0表示没有指定任务。 */
    uint32_t active_task;       /* 软件发现故障时正在执行的关键任务编号，0表示未执行关键任务。 */
    uint32_t stage;             /* 软件发现故障时的IwdgStage_t，用于区分启动、定标和运行阶段。 */
} IwdgResetRecord_t;

struct task_s;
extern volatile IwdgResetRecord_t g_iwdg_last_reset; /* 调试器读取本次启动保存的复位记录，不占用业务串口。 */
void Iwdg_Init(void); /* HAL初始化后立即启用IWDG，并保存、清除上次复位标志。 */
void Iwdg_BootStage(IwdgStage_t stage); /* 进入下一个开机步骤；同一步骤不能反复调用来延长时间。 */
void Iwdg_BootPoll(void); /* 启动页完成一轮处理后检查总时限，再按100ms间隔喂狗。 */
void Iwdg_EnterCalibration(void); /* 只能从启动页进入脚踏定标阶段。 */
void Iwdg_CalibrationCycleComplete(void); /* 完整定标循环结束后调用，循环卡住时不会继续喂狗。 */
uint8_t Iwdg_RegisterTask(struct task_s *task); /* 按完整回调名称登记关键任务，返回固定编号，非关键任务返回0。 */
void Iwdg_TaskEnter(uint8_t task_id); /* 取得公共锁后记录正在执行的任务编号。 */
void Iwdg_TaskComplete(struct task_s *task); /* 回调正常返回后记录完成时刻，再清除正在执行的任务编号。 */
void IwdgTaskInit(void); /* 所有业务任务和公共锁创建后，创建不使用公共锁的看门狗检查线程。 */
uint8_t Iwdg_WasAbnormalReset(void); /* 看门狗或软件复位返回1；普通上电返回0，用于要求控制源重新松开。 */
void Iwdg_ForceReset(IwdgFault_t reason, uint32_t task_id); /* 保存故障后复位；不等待业务锁、串口或EEPROM。 */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */

