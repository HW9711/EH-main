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
// 通讯协议定义
//==============================================================================
#define FOOT_FRAME_HEADER   0xAA    //帧头
#define FOOT_FRAME_TAIL     0xBB    //帧尾
#define FOOT_BUFFER_SIZE    11      //数据缓冲区大小

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
// 离线检测阈值 (25ms周期, 约500ms)
 //==============================================================================
#define FOOT_OFFLINE_THRESHOLD  20  //离线阈值计数

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

/* 脚踏诊断快照只提供只读字段，诊断模块不得直接访问 sscFOOT.c 内部 static 状态。 */
typedef struct
{
    uint16_t frame_sequence;       /* 每份 CRC 正确 UART4 实时帧递增一次，自然回绕。 */
    uint8_t pedal_type;            /* 1 单踏板、2 双段、3 双脚踏。 */
    uint8_t runtime_frame_valid;    /* 250ms 实时帧门禁当前是否有效。 */
    uint16_t adc_left;             /* 单踏板/双段实时值或双脚踏左值。 */
    uint16_t adc_right;            /* 双脚踏右值，其它类型为 0。 */
    uint16_t low_left;             /* 左侧低点定标。 */
    uint16_t mid_left;             /* 左侧中点定标。 */
    uint16_t high_left;            /* 左侧高点定标。 */
    uint16_t low_right;            /* 右侧低点定标。 */
    uint16_t mid_right;            /* 右侧中点定标。 */
    uint16_t high_right;           /* 右侧高点定标。 */
    uint8_t stop_latched;          /* 独立脚踏停机锁存。 */
    uint8_t release_ready;         /* 已观察到释放，等待下一次有效踩下。 */
    uint8_t active_source;         /* 0 无、1 左、2 右。 */
} FootTraceSnapshot_t;

//==============================================================================
// 函数声明
//==============================================================================
/**
 * @brief 数据解析函数
 * @brief 解析脚踏板原始数据，提取有用信息
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
 * 函数功能：复制脚踏实时帧、定标值和停机门禁的一致性只读快照。
 * 输入参数：snapshot 指向调用方提供的快照缓存。
 * 返回参数：复制成功返回 1，空指针返回 0。
 */
uint8_t Foot_CopyTraceSnapshot(FootTraceSnapshot_t *snapshot);

/*
 * 函数功能：判断当前可信脚踏 ADC 是否已经退出手柄电机运行区。
 * 输入参数：无。
 * 返回参数：已在释放区返回 1；帧无效、定标不完整或仍在运行区返回 0。
 */
uint8_t Foot_TraceCurrentAdcIsInMotorReleaseRegion(void);

/*
 * 函数功能：查询脚踏手柄电机是否处于独立停止锁存状态。
 * 输入参数：无。
 * 返回参数：锁存有效返回1，当前踩踏周期已经授权运行返回0。
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
