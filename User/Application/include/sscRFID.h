#ifndef __SSC_RFID_H
#define __SSC_RFID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RFID_PAYLOAD_EPC_LENGTH     12U  /* EPC 模式按协议提取 data[8..19]，用于公共接头刀具头。 */
#define RFID_PAYLOAD_USER_LENGTH    16U  /* USER 模式按协议提取 data[20..35]，用于 PXBA/PXBB 分体式刀具头。 */
#define RFID_PAYLOAD_MAX_LENGTH     RFID_PAYLOAD_USER_LENGTH /* 结果缓存按较大的 USER 数据区预留空间。 */

typedef enum
{
    RFID_READ_SOURCE_NONE = 0U, /* 无 RFID 读取来源，用于清空或无效请求。 */
    RFID_READ_SOURCE_EPC = 1U,  /* 公共接头式可拆手柄读取 EPC 区。 */
    RFID_READ_SOURCE_USER = 2U  /* 分体式 PXBA/PXBB 可拆手柄读取 USER 区。 */
} RfidReadSource_t;

typedef struct
{
    bool valid;                                      /* true 表示本结构保存了一次校验通过的 RFID 标签结果。 */
    bool cache_hit;                                  /* true 表示本次标签数据与该通道上一次有效标签完全一致。 */
    bool changed;                                    /* true 表示本次标签数据与该通道上一次有效标签不同。 */
    uint8_t channel;                                 /* 结果所属通道，使用 CHANNEL_A/CHANNEL_B 的数值。 */
    RfidReadSource_t source;                         /* 结果来源，区分 EPC 公共接头和 USER 分体式。 */
    uint8_t payload_length;                          /* payload 实际长度，EPC 为 12，USER 为 16。 */
    uint8_t payload[RFID_PAYLOAD_MAX_LENGTH];        /* 按协议提取出的完整原始标签数据，用于缓存比较。 */
    uint16_t sequence;                               /* 每次成功读到标签后递增，handlescan 用它判断是否有新结果。 */
    uint16_t presence_sequence;                      /* 每次有效读到 RFID 标签都递增，在线监测用它区分“同一标签仍在”和“连续读不到标签”。 */
} RfidToolResult_t;

void SscSplitTypeAutoModeGetData_Init(void);
void SscRadioFreq_Init(void);

bool Rfid_RequestToolRead(uint8_t channel, RfidReadSource_t source, bool fast_mode);
bool Rfid_CopyLastResult(uint8_t channel, RfidToolResult_t *result);
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
