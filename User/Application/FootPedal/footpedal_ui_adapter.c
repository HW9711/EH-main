#include "footpedal_ui_adapter.h"

typedef struct
{
    uint8_t select_window_num;
    uint8_t select_window_num_last;
    uint16_t window_disappear_time_cnt;
} FootPedalUiState_t;

static FootPedalUiState_t s_foot_pedal_ui_state = {0U, 0U, 0U};

/*
 * 旧脚踏状态机下线后，当前工程仍只需要“切换手柄时复位脚踏选框”这一项 UI 辅助能力。
 * 这里把它收敛到独立适配文件，避免继续把整份旧脚踏状态机和历史屏幕数据仓库带进构建。
 */
void FootPedal_SelectWin(uint8_t num)
{
    if (s_foot_pedal_ui_state.select_window_num == 0U)
    {
        return;
    }

    if (num == 0xFFU)
    {
        /* 新屏主运行页没有旧脚踏选框 VP，这里只复位本地选框状态，不再写旧屏窗口接口。 */
        s_foot_pedal_ui_state.select_window_num_last = 0U;
        s_foot_pedal_ui_state.select_window_num = 0U;
    }
    else if (s_foot_pedal_ui_state.select_window_num != 1U)
    {
        s_foot_pedal_ui_state.select_window_num = 1U;
        /* 新屏脚踏选框由主运行页控制模式图标表达，旧窗口切换资源已删除。 */
        s_foot_pedal_ui_state.select_window_num_last = s_foot_pedal_ui_state.select_window_num;
    }

    s_foot_pedal_ui_state.window_disappear_time_cnt = 0U;
}
