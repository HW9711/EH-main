#ifndef __MAINBOARD_SOFTWARE_VERSION_H
#define __MAINBOARD_SOFTWARE_VERSION_H

#include <stdint.h>

#define MAINBOARD_SW_VERSION_BASE_TEXT       "V1.0"       /* 对外显示的主控大版本号，最多保存8字节；改后影响HEX标记和EEPROM版本记录，不控制业务功能。 */
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

/* 驱动版本ID：1手柄、2逻辑A泵、3逻辑B泵；分别独占主控AT24C32 Page2/3/4。 */
uint8_t DriverSoftwareVersion_Request(uint8_t id); /* 开始异步查询；忙时拒绝，不复用历史记录。 */
void DriverSoftwareVersion_Service(uint8_t idle); /* 只在停止状态推进查询和EEPROM写回。 */
uint8_t DriverSoftwareVersion_Poll(uint8_t *payload); /* 取走固定33字节结果并释放事务。 */
uint8_t DriverSoftwareVersion_TakePumpQuery(uint8_t channel, uint8_t *frame); /* 在对应泵安全周期领取六字节查询。 */
uint16_t DriverSoftwareVersion_CapturePump(uint8_t channel, const uint8_t *data, uint16_t length); /* 版本帧整段隔离，不交给速度反馈解析。 */
#endif
