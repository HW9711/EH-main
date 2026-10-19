#ifndef __SSC_RFID_H
#define __SSC_RFID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RFID_PAYLOAD_EPC_LENGTH     12U  /* 刀具标签数据长 12 字节，取回包的 data[8..19]；公共接头和 PXBA/PXBB 共用此格式，不能单独改长度。 */
#define RFID_PAYLOAD_USER_LENGTH    16U /* 老USER包含14字节业务数据及高字节在前的CRC16。 */
#define RFID_PAYLOAD_MAX_LENGTH     RFID_PAYLOAD_USER_LENGTH /* 最大缓存16字节，兼容新版EPC12字节与老USER16字节。 */

#define RFID_TOOL_MODEL_PLANER            0x01U /* 标签刀具类型0x01：普通刨刀，可选正转、反转和往复；默认往复频率指令值为40，主控不换算成Hz。 */
#define RFID_TOOL_MODEL_GRINDER           0x02U /* 标签刀具类型 0x02：普通磨刀，只允许单方向连续转动，不做正反交替转动。 */
#define RFID_TOOL_MODEL_REVERSE_ROTATION  0x03U /* 标签刀具类型0x03：反旋机械刀具；标签指定默认方向，屏幕可切换正反转，不允许往复；发给电机的方向与屏幕相反。 */
#define RFID_TOOL_MODEL_MXYTP             0x04U /* 标签刀具类型 0x04：MXYTP；屏幕固定显示往复，电机一直正转，由刀具机械结构产生往复动作。 */
#define RFID_TOOL_MODEL_MXYTM             0x05U /* 标签刀具类型 0x05：MXYTM 单向磨刀；屏幕显示和电机转向都按标签设置。 */
#define RFID_TOOL_MODEL_REVERSE_PLANER    0x06U /* 标签刀具类型 0x06：反向刨刀，支持往复；选正转或反转连续运行时，发给电机的方向与所选方向相反。 */

typedef enum
{
    RFID_READ_SOURCE_NONE = 0U, /* 无 RFID 读取来源，用于清空或无效请求。 */
    RFID_READ_SOURCE_EPC = 1U,  /* 每次请求先读取新版CRC8 EPC，失败后在同一请求内读取USER。 */
    RFID_READ_SOURCE_USER = 2U  /* 老款16字节USER，仅在EPC识别失败后由任务读取。 */
} RfidReadSource_t;

typedef struct
{
    uint8_t model; /* 归一后的RFID型号1~6，供机械转向转换使用。 */
    uint8_t direction; /* 默认显示方向：0正转、1反转、2往复，不是驱动最终方向。 */
    uint8_t direction_mask; /* bit0正转、bit1反转、bit2往复；识别成功后用于限制换向。 */
    uint16_t diameter; /* 新EPC保留直径原值，老USER按原协议乘5。 */
    uint8_t length; /* 刀具长度，沿用各标签原协议。 */
    uint8_t angle; /* 刀具弯曲角度，沿用各标签原协议。 */
    uint32_t min_speed; /* 刀头最低转速，单位rpm；EPC的0值由PX普通刨刀适配500rpm。 */
    uint32_t max_speed; /* 刀头最高转速，单位rpm。 */
    uint32_t default_speed; /* 刀头默认转速，单位rpm，已检查在上下限内。 */
    uint32_t ratio; /* 减速比x100存低16位，增速比x100存高16位；老USER按兼容表补齐430或190。 */
    uint16_t current; /* 驱动电流阈值，单位0.01A；新EPC读取标签，老USER按PXYTP/PXYTM兼容表补齐。 */
    uint16_t speed_step_small; /* 老USER兼容表给出小步进，单位rpm；新版EPC保持0并沿用原步进处理。 */
    uint16_t speed_step_large; /* 老USER兼容表给出大步进，单位rpm；新版EPC保持0并沿用原步进处理。 */
    uint8_t flow; /* 默认注水流量，单位mL/min。 */
    uint8_t frequency; /* 往复频率指令原值，老USER取B10，新刨刀默认40。 */
} RfidToolParameters_t;

bool Rfid_DecodeToolPayload(RfidReadSource_t source, const uint8_t *payload, uint8_t length, RfidToolParameters_t *parameters);

typedef enum
{
    RFID_REQUEST_RESULT_UNKNOWN = 0U,  /* 找不到此请求编号，或记录已被新请求覆盖；不能据此认定刀具标签丢失。 */
    RFID_REQUEST_RESULT_PENDING,       /* 请求正在排队或读取中；手柄扫描程序必须继续等待结果。 */
    RFID_REQUEST_RESULT_SUCCESS,       /* EPC或USER已经通过模块校验、业务CRC和参数检查。 */
    RFID_REQUEST_RESULT_TIMEOUT,       /* 重试次数用完或等待超时，仍未读到完整、校验正确的 EPC 标签数据。 */
    RFID_REQUEST_RESULT_CANCELED       /* 因电机启动、通道状态变化或刀具信息被清除而取消；不算读不到标签。 */
} RfidRequestResult_t;

typedef struct
{
    bool valid;                                      /* true 表示本结构保存了一次校验通过的 RFID 标签结果。 */
    bool cache_hit;                                  /* true 表示本次标签数据与该通道上一次有效标签完全一致。 */
    bool changed;                                    /* true 表示本次标签数据与该通道上一次有效标签不同。 */
    uint8_t channel;                                 /* 结果所属通道，使用 CHANNEL_A/CHANNEL_B 的数值。 */
    RfidReadSource_t source;                         /* 成功读取的数据区为EPC或USER；清空结果时为NONE。 */
    uint8_t payload_length;                          /* EPC为12字节，老USER为16字节，比较缓存前必须检查来源和长度。 */
    uint8_t payload[RFID_PAYLOAD_MAX_LENGTH];        /* 按协议提取出的完整原始标签数据，用于缓存比较。 */
    bool tag_epc_valid; /* 本帧已通过模块校验且含12字节标签标识；用于核对EPC和USER属于同一标签。 */
    uint8_t tag_epc[RFID_PAYLOAD_EPC_LENGTH]; /* USER回包附带的EPC，或库存回包中的EPC原文。 */
    uint16_t sequence;                               /* 刀具数据需要重新处理时才增加；普通轮询读到相同标签时不变，手柄扫描程序用它判断是否需要刷新。 */
    uint16_t presence_sequence;                      /* 每次读到有效标签都增加，即使标签内容没变；用来确认刀具仍能被读到。 */
} RfidToolResult_t;

typedef struct
{
    uint32_t request_count;                          /* 已发送的EPC和USER读取命令总数，A/B通道分别累计。 */
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
