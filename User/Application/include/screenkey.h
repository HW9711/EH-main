//screenkey.h

#ifndef __SCREENKEY_H
#define __SCREENKEY_H

#include <stdint.h>

// 一次性按键事件的空值，保持与历史屏幕键值一致，避免定标页误处理无按键状态。
#ifndef KEY_NONE
#define KEY_NONE 0xFFU
#endif

// 屏幕启动页连续点击事件，数值保持不变，用于进入隐藏启动流程。
#ifndef KEY_CONTINUOUSCLICK
#define KEY_CONTINUOUSCLICK 21U
#endif

// 脚踏定标页存储低点/高点/中点的事件值，来自屏幕串口按键解析。
#ifndef KEY_STORAGEMIN
#define KEY_STORAGEMIN 15U
#endif

#ifndef KEY_STORAGEMAX
#define KEY_STORAGEMAX 16U
#endif

#ifndef KEY_STORAGEMIN2
#define KEY_STORAGEMIN2 80U
#endif

#ifndef KEY_STORAGEMAX2
#define KEY_STORAGEMAX2 81U
#endif

#ifndef KEY_STORAMEDIAN
#define KEY_STORAMEDIAN 82U
#endif

#ifndef KEY_STORAMEDIAN2
#define KEY_STORAMEDIAN2 83U
#endif

// 脚踏板物理按键桥接到定标页的事件值，保持旧协议数值不变。
#ifndef M_KEY_FOOT
#define M_KEY_FOOT 85U
#endif

#ifndef L_KEY_FOOT
#define L_KEY_FOOT 86U
#endif

#ifndef R_KEY_FOOT
#define R_KEY_FOOT 87U
#endif

//============================================================================
//屏”长按键“任务初始化
//============================================================================
// 屏幕启动页和脚踏定标页通过一次性事件缓存取键，避免继续写旧全局键值。
void ScreenKey_LegacyEventPost(uint8_t key_value);
uint8_t ScreenKey_LegacyEventTake(void);

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



