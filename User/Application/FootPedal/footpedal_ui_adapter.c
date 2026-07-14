#include "footpedal_ui_adapter.h"

typedef struct
{
    uint8_t select_window_num;           /* 当前旧脚踏选框编号；0 表示新屏没有待复位选框。 */
    uint8_t select_window_num_last;      /* 上一次选框编号，仅用于兼容旧调用方的状态复位。 */
    uint16_t window_disappear_time_cnt;  /* 旧选框消失计时；每次有效切换后从 0 重新累计。 */
} FootPedalUiState_t;

static FootPedalUiState_t s_foot_pedal_ui_state = {0U, 0U, 0U};

/*
 * 函数功能：兼容旧脚踏选框复位调用，新屏只维护本地状态，不再发送已删除的窗口 VP。
 * 输入参数：num 为旧选框目标编号；0xFF 表示退出并复位选框。
 * 返回参数：无。
 */
void FootPedal_SelectWin(uint8_t num)
{
    if (s_foot_pedal_ui_state.select_window_num == 0U) /* 当前没有旧选框时无需重复复位，避免无效状态写入。 */
    {
        return;
    }

    if (num == 0xFFU) /* 0xFF 表示切换手柄前退出旧选框，必须清空当前和上次编号。 */
    {
        /* 新屏主运行页没有旧脚踏选框 VP，这里只复位本地选框状态，不再写旧屏窗口接口。 */
        s_foot_pedal_ui_state.select_window_num_last = 0U;
        s_foot_pedal_ui_state.select_window_num = 0U;
    }
    else if (s_foot_pedal_ui_state.select_window_num != 1U) /* 兼容入口只保留主选框 1，重复选择不改状态。 */
    {
        s_foot_pedal_ui_state.select_window_num = 1U;
        /* 新屏脚踏选框由主运行页控制模式图标表达，旧窗口切换资源已删除。 */
        s_foot_pedal_ui_state.select_window_num_last = s_foot_pedal_ui_state.select_window_num;
    }

    s_foot_pedal_ui_state.window_disappear_time_cnt = 0U;
}
