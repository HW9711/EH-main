#ifndef CONTROL_ARBITRATION_H
#define CONTROL_ARBITRATION_H

#include <stdbool.h>
#include <stdint.h>

/* 记录当前由谁控制手柄电机。这些值是固定编号，不是功能开关；本地单独操作泵不占用电机控制权。 */
#define CONTROL_OWNER_NONE      0U /* 无人占用，允许新的控制来源申请。 */
#define CONTROL_OWNER_EXTERNAL  1U /* 外部电脑已取得控制权，退出或超时前不让本地启停。 */
#define CONTROL_OWNER_FOOT      2U /* 脚踏正在控制手柄电机。 */
#define CONTROL_OWNER_SCREEN    3U /* 屏幕触控正在控制手柄电机。 */
#define CONTROL_OWNER_HANDLE    4U /* 手柄按键正在控制手柄电机。 */

bool ControlArbitration_IsExternalActive(void); /* 查询外控是否持有独占控制权。 */
uint8_t ControlArbitration_GetLocalDriveTypeAfterExit(void); /* 外控/触控退出后按脚踏、手柄可用状态选择本地方式。 */
uint8_t ControlArbitration_GetOwnerByPumpKey(uint8_t key_value); /* 把泵业务按键来源转换成控制权编号。 */
bool ControlArbitration_IsOwner(uint8_t owner); /* 判断指定来源是否为当前控制源。 */
bool ControlArbitration_IsBusyByOther(uint8_t owner); /* 判断是否被其它来源占用。 */
bool ControlArbitration_TryEnter(uint8_t owner); /* 尝试取得控制权，不强行抢占其它来源。 */
void ControlArbitration_Exit(uint8_t owner); /* 指定来源主动退出控制。 */
void ControlArbitration_ExitLocalControlIfIdle(uint8_t owner); /* 本地电机停稳后释放控制权。 */
void ControlArbitration_RefreshMotorOwner(void); /* 周期维护电机控制权和限时报警。 */
void ControlArbitration_ForceRelease(void); /* 只清除控制来源；停电机、停泵必须由调用方另行处理。 */
bool ControlArbitration_EnterExternalControl(void); /* 申请外部独占控制。 */
void ControlArbitration_ReleaseExternalControl(void); /* 停止外控输出并恢复本地控制方式。 */
bool ControlArbitration_ShouldBlockLocalKey(uint8_t control_type, uint8_t control_key); /* 判断本地按键是否应被当前控制权拦截。 */

#endif
