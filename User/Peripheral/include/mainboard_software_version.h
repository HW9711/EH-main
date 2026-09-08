#ifndef __MAINBOARD_SOFTWARE_VERSION_H
#define __MAINBOARD_SOFTWARE_VERSION_H

#include <stdint.h>

#define MAINBOARD_SW_VERSION_BASE_TEXT       "V1.1"       /* 对外显示的主控大版本号，最多保存8字节；改后影响HEX标记和EEPROM版本记录，不控制业务功能。 */
#define MAINBOARD_SW_VERSION_CODE_START      0x08000000UL /* 固件版本校验固定从 Flash 起始地址开始。 */
#define MAINBOARD_SW_VERSION_CODE_SIZE       0x00040000UL /* 参与固件CRC32计算的字节数：当前前256KB；改后上位机也须按同一范围计算，其余Flash不参与此校验。 */
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
  MAINBOARD_SW_VERSION_STATUS_MAGIC_FAIL = 5U,       /* Page1开头不是固定标记SWV1，不能按软件版本记录读取。 */
  MAINBOARD_SW_VERSION_STATUS_FORMAT_FAIL = 6U       /* Page1 记录格式版本、算法或校验范围不匹配当前固件。 */
} MainboardSoftwareVersionStatus_t;

uint8_t MainboardSoftwareVersion_Sync(void);

uint8_t MainboardSoftwareVersion_ReadPayload(uint8_t *payload, uint16_t payload_size, uint16_t *payload_len);

#endif
