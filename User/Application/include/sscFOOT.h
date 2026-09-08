/**
 ******************************************************************************
 * @file    sscFOOT.h
 * @brief   脚踏板数据解析驱动头文件
 ******************************************************************************
 */
 
#ifndef __SSCFOOT_H__
#define __SSCFOOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//==============================================================================
// 旧 AA/BB 协议定义；当前 sscFOOT.c 使用 FE EF 协议，不使用下面三个宏。
//==============================================================================
#define FOOT_FRAME_HEADER   0xAA    // 旧协议帧头固定值，修改它不会改变当前 FE EF 解析。
#define FOOT_FRAME_TAIL     0xBB    // 旧协议帧尾固定值，不是当前脚踏的可调业务参数。
#define FOOT_BUFFER_SIZE    11      // 旧接口缓冲区字节数，不决定当前 10/18/24 字节帧长度。

//==============================================================================
// 解析错误码定义
//==============================================================================
#define FOOT_SUCCESS        0x00    //解析成功
#define FOOT_ERR_PARAM      0x01    //参数错误
#define FOOT_ERR_FRAME      0x02    //帧格式错误
#define FOOT_ERR_CHECKSUM   0x03    //校验和错误
#define FOOT_ERR_TIMEOUT    0x04    //超时错误

//==============================================================================
// 按键状态定义
//==============================================================================
#define FOOT_KEY_NONE       0x00    //无按键
#define FOOT_KEY_LEFT_SHORT 0x03    //左边短按
#define FOOT_KEY_RIGHT_SHORT 0x04   //右边短按
#define FOOT_KEY_RIGHT_LONG  0x05    //右边长按
#define FOOT_KEY_CENTER_SHORT 0x02  //中间短按
#define FOOT_KEY_CENTER_LONG  0x01   //中间长按(手柄切换)

//==============================================================================
// 连接状态定义
//==============================================================================
#define FOOT_CONNECTED      0x01    //已连接
#define FOOT_DISCONNECTED   0x00    //未连接

//==============================================================================
// 脚踏板类型定义
//==============================================================================
#define FOOT_TYPE_SINGLE    0x01    //单踏板
#define FOOT_TYPE_DOUBLE    0x02    //双踏板

//==============================================================================
// 按键类型定义
//==============================================================================
typedef enum {
    FOOT_PRESS_NONE = 0,    //无按键
    FOOT_PRESS_SHORT,       //短按
    FOOT_PRESS_LONG         //长按
} FootPressType_t;

typedef enum {
    FOOT_KEY_TYPE_NONE = 0,     //无按键
    FOOT_KEY_TYPE_LEFT,         //左边按键
    FOOT_KEY_TYPE_RIGHT,        //右边按键
    FOOT_KEY_TYPE_CENTER       //中间按键
} FootKeyType_t;

//==============================================================================
// 消息类型定义
//==============================================================================
#define FOOT_MSG_CONNECT    1      //连接状态消息
#define FOOT_MSG_KEY        2      //按键状态消息
#define FOOT_MSG_AD         3      //AD值更新消息

//==============================================================================
// 旧接口离线计数；当前解析使用 sscFOOT.c 中的 FOOT_RUNTIME_TIMEOUT_TICKS 和 FOOT_OFFLINE_TIMEOUT_TICKS。
 //==============================================================================
#define FOOT_OFFLINE_THRESHOLD  20  // 历史设置为 20 次、每次 25ms；当前未使用，修改它不改变实际停机或掉线时间。

//==============================================================================
// 解析后数据结构体
//==============================================================================
typedef struct {
    uint16_t pedalADValue;       //脚踏板AD值
    uint8_t keyStatus;           //按键状态
    uint8_t connectStatus;       //连接状态
    uint8_t pedalType;           //脚踏板类型
    
    //解析后的详细信息
    uint8_t isConnected;        //是否连接标志
    uint8_t isDoublePedal;       //是否双踏板标志
    
    FootKeyType_t keyType;       //按键类型
    FootPressType_t keyPressType; //按键按压类型
    
    uint8_t offlineCounter;      //离线计数
} FootParsedData_t;

//==============================================================================
// 函数声明
//==============================================================================
/**
 * @brief 旧 AA/BB 解析接口声明；当前 UART4 任务使用 sscFOOT.c 中的 Foot_ParseDataS。
 * 
 * 协议格式:
 * +------+------+-------+--------+--------+--------+--------+------+----------+
 * | 帧头  | 类型  | AD_H  | AD_L  | 按键   | 连接   | 类型   | 校验和 | 帧尾   |
 * | 0xAA  | 0x01  | 0xXX  | 0xXX  | 0xXX   | 0xXX   | 0xXX   | 0xXX  | 0xBB   |
 * +------+------+-------+--------+--------+--------+--------+------+----------+
 *
 * @param rawData 原始数据缓冲区
 * @param dataLen 数据长度
 * @param output 解析后数据输出结构体
 * @return 解析成功标志 (0=成功, 其他=失败原因)
 */
uint8_t Foot_ParseData(uint8_t* rawData, uint8_t dataLen, FootParsedData_t* output);

/**
 * @brief 获取解析后的数据
 * @return 解析数据指针
 */
FootParsedData_t* Foot_GetParsedData(void);

/**
 * @brief 清除按键状态
 */
void Foot_ClearKeyStatus(void);

/*
 * 函数功能：查询脚踏是否要求手柄电机保持停止。
 * 输入参数：无。
 * 返回参数：1 表示必须停止；0 表示本次踩踏已允许运行。
 */
uint8_t Foot_IsMotorStopLatched(void);

/**
 * @brief 脚踏板任务初始化
 */
void SscFootControlTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SSCFOOT_H__ */
