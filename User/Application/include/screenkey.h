//screenkey.h

#ifndef __SCREENKEY_H
#define __SCREENKEY_H

#include <stdint.h>

/* 触控按住信号的有效期，单位 ms：默认 420。0x5520 成功放入按键队列后开始计时；到期未收到新信号就停机，改大将延长松手后的最长停机等待。 */
#define SCREENKEY_TOUCH_KEEPALIVE_TIMEOUT_MS 420U

// 一次性按键事件的空值，保持与历史屏幕键值一致，避免定标页误处理无按键状态。
#ifndef KEY_NONE
#define KEY_NONE 0xFFU
#endif

// 屏幕启动页连续点击事件，数值保持不变，用于进入隐藏启动流程。
#ifndef KEY_CONTINUOUSCLICK
#define KEY_CONTINUOUSCLICK 21U
#endif

// 以下是脚踏定标页的保存按钮编号，不是定标数值；必须与屏幕按键解析一致。
#ifndef KEY_STORAGEMIN
#define KEY_STORAGEMIN 15U // 保存单踏板或左踏板的低点。
#endif

#ifndef KEY_STORAGEMAX
#define KEY_STORAGEMAX 16U // 保存单踏板或左踏板的高点。
#endif

#ifndef KEY_STORAGEMIN2
#define KEY_STORAGEMIN2 80U // 保存双脚踏右侧的低点。
#endif

#ifndef KEY_STORAGEMAX2
#define KEY_STORAGEMAX2 81U // 保存双脚踏右侧的高点。
#endif

#ifndef KEY_STORAMEDIAN
#define KEY_STORAMEDIAN 82U // 保存双段或双脚踏左侧的中点，普通单踏板不使用。
#endif

#ifndef KEY_STORAMEDIAN2
#define KEY_STORAMEDIAN2 83U // 保存双脚踏右侧的中点。
#endif

// 脚踏板实体键在定标页使用的编号，只切换调试显示，不保存定标值。
#ifndef M_KEY_FOOT
#define M_KEY_FOOT 85U // 中间实体键。
#endif

#ifndef L_KEY_FOOT
#define L_KEY_FOOT 86U // 左侧实体键。
#endif

#ifndef R_KEY_FOOT
#define R_KEY_FOOT 87U // 右侧实体键。
#endif

//============================================================================
//屏”长按键“任务初始化
//============================================================================
// 屏幕启动页和脚踏定标页通过一次性事件缓存取键，避免继续写旧全局键值。
void ScreenKey_LegacyEventPost(uint8_t key_value);
uint8_t ScreenKey_LegacyEventTake(void);

/*
 * 函数功能：检查最近一次已放入按键队列的“仍在按住”信号是否还有效。
 * 输入参数：无，读取当前HAL毫秒时钟和最近接受时刻。
 * 返回参数：1 表示未到 420ms；0 表示没有有效信号、信号已取消或已到 420ms。
 */
uint8_t ScreenKey_IsAcceptedTouchKeepAliveFresh(void);

//============================================================================
//接收串口数据任务，收到的数据放入缓冲区
//    帧头  | 长度 | 指令 | 变量地址  | 读出长度 | 读出的数据
//0x5A 0xA5 | 0x06 | 0x83 | 0x10 0x03 |   0x01   | 0x00 0x1F
//============================================================================
void ScreenKey_Scan(void);

//============================================================================
//屏”按键“串口接收的任务初始化
//============================================================================
void ScreenKey_ScanInit(void);

#endif  //__SCREENKEY_H



