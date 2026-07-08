#ifndef __MAINBOARD_SOFTWARE_VERSION_H
#define __MAINBOARD_SOFTWARE_VERSION_H

#include <stdint.h>

#define MAINBOARD_SW_VERSION_BASE_TEXT       "V1.0"       /* 主控固定大版本号，升级大版本时只改这里。 */
#define MAINBOARD_SW_VERSION_CODE_START      0x08000000UL /* 固件版本校验固定从 Flash 起始地址开始。 */
#define MAINBOARD_SW_VERSION_CODE_SIZE       0x00040000UL /* 固件版本校验固定覆盖前 256KB，后 768KB 预留参数/日志/升级缓存。 */
#define MAINBOARD_SW_VERSION_RECORD_SIZE     32U          /* 主控板 AT24C32 Page1 固定 32 字节，只存软件版本记录。 */
#define MAINBOARD_SW_VERSION_TEXT_SIZE       8U           /* 固定大版本字符串占 8 字节，短字符串后续补 0。 */
#define MAINBOARD_SW_VERSION_PAYLOAD_SIZE    (MAINBOARD_SW_VERSION_RECORD_SIZE + 1U) /* 外部通信载荷：状态 1 字节 + Page1 原始记录 32 字节。 */

typedef enum
{
  MAINBOARD_SW_VERSION_STATUS_OK = 0U,               /* Page1 记录有效，且字段校验通过。 */
  MAINBOARD_SW_VERSION_STATUS_BAD_PARAM = 1U,        /* 调用参数为空或缓存长度不足。 */
  MAINBOARD_SW_VERSION_STATUS_EEPROM_READ_FAIL = 2U, /* 主控板 AT24C32 未应答或读取失败。 */
  MAINBOARD_SW_VERSION_STATUS_EEPROM_WRITE_FAIL = 3U,/* 主控板 AT24C32 写入或写后读回失败。 */
  MAINBOARD_SW_VERSION_STATUS_PAGE_SUM_FAIL = 4U,    /* Page1 最后 2 字节页校验和不匹配。 */
  MAINBOARD_SW_VERSION_STATUS_MAGIC_FAIL = 5U,       /* Page1 魔术字不是 SWV1，说明不是软件版本记录。 */
  MAINBOARD_SW_VERSION_STATUS_FORMAT_FAIL = 6U       /* Page1 记录格式版本、算法或校验范围不匹配当前固件。 */
} MainboardSoftwareVersionStatus_t;

uint8_t MainboardSoftwareVersion_Sync(void);

uint8_t MainboardSoftwareVersion_ReadPayload(uint8_t *payload, uint16_t payload_size, uint16_t *payload_len);

#endif
