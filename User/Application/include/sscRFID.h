#ifndef __SSC_RFID_H
#define __SSC_RFID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RFID_PAYLOAD_EPC_LENGTH     12U  /* EPC 模式按协议提取 data[8..19]，公共接头和 PXBA/PXBB 均只使用这一种射频数据区。 */
#define RFID_PAYLOAD_MAX_LENGTH     RFID_PAYLOAD_EPC_LENGTH /* 当前最终协议只保留 EPC，结果缓存按 12 字节预留。 */

#define RFID_TOOL_MODEL_PLANER            0x01U /* 普通刨刀具，支持正转、反转和电气往复，默认频率为 4Hz。 */
#define RFID_TOOL_MODEL_GRINDER           0x02U /* 普通磨刀具，只允许单向旋转模式，不进入电气往复。 */
#define RFID_TOOL_MODEL_REVERSE_ROTATION  0x03U /* 反旋机械刀具，屏幕方向服从标签，驱动实际方向与屏幕方向相反。 */
#define RFID_TOOL_MODEL_MXYTP             0x04U /* MXYTP 机械往复刀具，屏幕固定往复，驱动实际固定正转。 */
#define RFID_TOOL_MODEL_MXYTM             0x05U /* MXYTM 单向磨刀具，屏幕和驱动方向均服从标签。 */
#define RFID_TOOL_MODEL_REVERSE_PLANER    0x06U /* 反向刨刀具，默认可往复；选择正转或反转单向运行时驱动实际方向取反。 */

typedef enum
{
    RFID_READ_SOURCE_NONE = 0U, /* 无 RFID 读取来源，用于清空或无效请求。 */
    RFID_READ_SOURCE_EPC = 1U   /* 公共接头和 PXBA/PXBB 可拆手柄统一读取 EPC 区。 */
} RfidReadSource_t;

typedef enum
{
    RFID_REQUEST_RESULT_UNKNOWN = 0U,  /* 请求票据不存在或已经被后续请求覆盖，调用方不能据此累计缺失。 */
    RFID_REQUEST_RESULT_PENDING,       /* 请求已经入队或正在执行，handlescan 必须继续等待明确终态。 */
    RFID_REQUEST_RESULT_SUCCESS,       /* 请求已经收到并解析出完整合法 EPC。 */
    RFID_REQUEST_RESULT_TIMEOUT,       /* 请求在限定尝试次数和硬超时内没有取得合法 EPC。 */
    RFID_REQUEST_RESULT_CANCELED       /* 请求因运行、模式、通道或代次变化被业务取消，不属于射频缺失。 */
} RfidRequestResult_t;

typedef struct
{
    bool valid;                                      /* true 表示本结构保存了一次校验通过的 RFID 标签结果。 */
    bool cache_hit;                                  /* true 表示本次标签数据与该通道上一次有效标签完全一致。 */
    bool changed;                                    /* true 表示本次标签数据与该通道上一次有效标签不同。 */
    uint8_t channel;                                 /* 结果所属通道，使用 CHANNEL_A/CHANNEL_B 的数值。 */
    RfidReadSource_t source;                         /* 结果来源，当前只允许 EPC；字段保留用于清空和有效性判断。 */
    uint8_t payload_length;                          /* payload 实际长度，当前固定为 EPC 12 字节。 */
    uint8_t payload[RFID_PAYLOAD_MAX_LENGTH];        /* 按协议提取出的完整原始标签数据，用于缓存比较。 */
    uint16_t sequence;                               /* 每次成功读到标签后递增，handlescan 用它判断是否有新结果。 */
    uint16_t presence_sequence;                      /* 每次有效读到 RFID 标签都递增，在线监测用它区分“同一标签仍在”和“连续读不到标签”。 */
} RfidToolResult_t;

typedef struct
{
    uint32_t request_count;                          /* 已发送的 EPC 读取命令总数，A/B 通道分别累计。 */
    uint32_t valid_response_count;                   /* 已收到并通过协议校验的有效回包总数。 */
    uint32_t lost_response_count;                    /* 下一条命令发送前仍未收到有效回包的已完成请求总数。 */
    uint32_t monitor_completion_count;               /* handlescan 在线监测已得到成功或未响应结论的累计次数。 */
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
