#ifndef __SSC_RFID_H
#define __SSC_RFID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RFID_PAYLOAD_EPC_LENGTH     12U  /* 刀具标签数据长 12 字节，取回包的 data[8..19]；公共接头和 PXBA/PXBB 共用此格式，不能单独改长度。 */
#define RFID_PAYLOAD_MAX_LENGTH     RFID_PAYLOAD_EPC_LENGTH /* 标签缓存容量等于 EPC 长度（12 字节）；若协议长度改变，解析和缓存必须一起检查。 */

#define RFID_TOOL_MODEL_PLANER            0x01U /* 标签刀具类型0x01：普通刨刀，可选正转、反转和往复；默认往复频率指令值为40，主控不换算成Hz。 */
#define RFID_TOOL_MODEL_GRINDER           0x02U /* 标签刀具类型 0x02：普通磨刀，只允许单方向连续转动，不做正反交替转动。 */
#define RFID_TOOL_MODEL_REVERSE_ROTATION  0x03U /* 标签刀具类型0x03：反旋机械刀具；标签指定默认方向，屏幕可切换正反转，不允许往复；发给电机的方向与屏幕相反。 */
#define RFID_TOOL_MODEL_MXYTP             0x04U /* 标签刀具类型 0x04：MXYTP；屏幕固定显示往复，电机一直正转，由刀具机械结构产生往复动作。 */
#define RFID_TOOL_MODEL_MXYTM             0x05U /* 标签刀具类型 0x05：MXYTM 单向磨刀；屏幕显示和电机转向都按标签设置。 */
#define RFID_TOOL_MODEL_REVERSE_PLANER    0x06U /* 标签刀具类型 0x06：反向刨刀，支持往复；选正转或反转连续运行时，发给电机的方向与所选方向相反。 */

typedef enum
{
    RFID_READ_SOURCE_NONE = 0U, /* 无 RFID 读取来源，用于清空或无效请求。 */
    RFID_READ_SOURCE_EPC = 1U   /* 公共接头和 PXBA/PXBB 可拆手柄统一读取 EPC 区。 */
} RfidReadSource_t;

typedef enum
{
    RFID_REQUEST_RESULT_UNKNOWN = 0U,  /* 找不到此请求编号，或记录已被新请求覆盖；不能据此认定刀具标签丢失。 */
    RFID_REQUEST_RESULT_PENDING,       /* 请求正在排队或读取中；手柄扫描程序必须继续等待结果。 */
    RFID_REQUEST_RESULT_SUCCESS,       /* 请求已经收到并解析出完整合法 EPC。 */
    RFID_REQUEST_RESULT_TIMEOUT,       /* 重试次数用完或等待超时，仍未读到完整、校验正确的 EPC 标签数据。 */
    RFID_REQUEST_RESULT_CANCELED       /* 因电机启动、通道状态变化或刀具信息被清除而取消；不算读不到标签。 */
} RfidRequestResult_t;

typedef struct
{
    bool valid;                                      /* true 表示本结构保存了一次校验通过的 RFID 标签结果。 */
    bool cache_hit;                                  /* true 表示本次标签数据与该通道上一次有效标签完全一致。 */
    bool changed;                                    /* true 表示本次标签数据与该通道上一次有效标签不同。 */
    uint8_t channel;                                 /* 结果所属通道，使用 CHANNEL_A/CHANNEL_B 的数值。 */
    RfidReadSource_t source;                         /* 记录数据是否来自 EPC 标签区；清空结果时改为 NONE，表示无有效来源。 */
    uint8_t payload_length;                          /* 标签数据实际长度，当前固定为 EPC 12 字节。 */
    uint8_t payload[RFID_PAYLOAD_MAX_LENGTH];        /* 按协议提取出的完整原始标签数据，用于缓存比较。 */
    uint16_t sequence;                               /* 刀具数据需要重新处理时才增加；普通轮询读到相同标签时不变，手柄扫描程序用它判断是否需要刷新。 */
    uint16_t presence_sequence;                      /* 每次读到有效标签都增加，即使标签内容没变；用来确认刀具仍能被读到。 */
} RfidToolResult_t;

typedef struct
{
    uint32_t request_count;                          /* 已发送的 EPC 读取命令总数，A/B 通道分别累计。 */
    uint32_t valid_response_count;                   /* 已收到并通过协议校验的有效回包总数。 */
    uint32_t lost_response_count;                    /* 下一条命令发送前仍未收到有效回包的已完成请求总数。 */
    uint32_t monitor_completion_count;               /* 手柄扫描程序在线监测已得到成功或未响应结论的累计次数。 */
    uint16_t invalid_frame_count;                    /* 收到数据但没有解析出有效 EPC 帧的异常批次总数，饱和后保持 65535。 */
    uint16_t confirmed_dropout_count;                /* 按现有缺失时间阈值确认的 RFID 刀具掉线次数，仅用于诊断统计。 */
} RfidLinkStatistics_t;

void SscSplitTypeAutoModeGetData_Init(void);
void SscRadioFreq_Init(void);

bool Rfid_RequestToolRead(uint8_t channel, RfidReadSource_t source, bool fast_mode);
bool Rfid_RequestToolReadTracked(uint8_t channel,
                                 RfidReadSource_t source,
                                 bool fast_mode,
                                 uint16_t *ticket);
RfidRequestResult_t Rfid_QueryRequestResult(uint8_t channel, uint16_t ticket);
bool Rfid_CopyLastResult(uint8_t channel, RfidToolResult_t *result);
bool Rfid_CopyLinkStatistics(uint8_t channel, RfidLinkStatistics_t *statistics);
void Rfid_RecordMonitorCompletion(uint8_t channel);
void Rfid_RecordConfirmedDropout(uint8_t channel);
bool Rfid_ParseReceivedFrame(const uint8_t *uartx_rf_buff,
                             uint16_t length,
                             RfidReadSource_t expected_source,
                             RfidToolResult_t *result);
void Rfid_ClearChannelResult(uint8_t channel);

void SendKeyRFIDMessageAup(uint8_t rfid_data);
void SendKeyRFIDMessageAdown(void);
void SendKeyRFIDMessageBup(uint8_t rfid_data);
void SendKeyRFIDMessageBdown(void);
void RfidHandle(uint8_t * uartx_rf_buff, uint8_t interface, bool enable_rfid, uint8_t rfid_type);

#endif
