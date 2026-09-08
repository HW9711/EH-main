#include "footpedal_ui_adapter.h"

typedef struct
{
    uint8_t select_window_num;           /* 旧脚踏选框的当前编号；0 表示没有选框需要清除。 */
    uint8_t select_window_num_last;      /* 旧脚踏选框的上次编号，清除选框时一并清零。 */
    uint16_t window_disappear_time_cnt;  /* 旧选框消失计时；每次有效切换后从 0 重新累计。 */
} FootPedalUiState_t;

static FootPedalUiState_t s_foot_pedal_ui_state = {0U, 0U, 0U};

/*
 * 函数功能：保留旧代码清除脚踏选框的入口；新屏没有该选框，只更新本地记录，不发送屏幕命令。
 * 输入参数：num 为旧选框目标编号；0xFF 表示退出并复位选框。
 * 返回参数：无。
 */
void FootPedal_SelectWin(uint8_t num)
{
    if (s_foot_pedal_ui_state.select_window_num == 0U) /* 没有旧选框需要处理，直接返回。 */
    {
        return;
    }

    if (num == 0xFFU) /* 0xFF 表示切换手柄前退出旧选框，必须清空当前和上次编号。 */
    {
        /* 新屏不使用旧选框地址，只清除本地记录，不给屏幕发送旧命令。 */
        s_foot_pedal_ui_state.select_window_num_last = 0U;
        s_foot_pedal_ui_state.select_window_num = 0U;
    }
    else if (s_foot_pedal_ui_state.select_window_num != 1U) /* 兼容入口只保留主选框 1，重复选择不改状态。 */
    {
        s_foot_pedal_ui_state.select_window_num = 1U;
        /* 新屏用脚控模式图标表示当前选择，这里不再切换旧选框。 */
        s_foot_pedal_ui_state.select_window_num_last = s_foot_pedal_ui_state.select_window_num;
    }

    s_foot_pedal_ui_state.window_disappear_time_cnt = 0U;
}
