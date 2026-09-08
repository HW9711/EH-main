#include "mainboard_software_version.h"
#include "iic.h"
#include <string.h>

#define MAINBOARD_SW_VERSION_PAGE1_ADDR        0x0000U     /* 主控板 AT24C32 Page1 起始地址，固定只存软件版本记录。 */
#define MAINBOARD_SW_VERSION_FORMAT            0x01U       /* Page1 软件版本记录格式版本。 */
#define MAINBOARD_SW_VERSION_CRC32_ALGO        0x01U       /* 代码区校验算法编号：1 表示标准 CRC32。 */
#define MAINBOARD_SW_VERSION_MAGIC0            'S'         /* Page1第0字节：固定标记SWV1的第1个字符，不是可调业务参数。 */
#define MAINBOARD_SW_VERSION_MAGIC1            'W'         /* Page1第1字节：固定标记的第2个字符。 */
#define MAINBOARD_SW_VERSION_MAGIC2            'V'         /* Page1第2字节：固定标记的第3个字符。 */
#define MAINBOARD_SW_VERSION_MAGIC3            '1'         /* Page1第3字节：固定标记的第4个字符。 */
#define MAINBOARD_SW_VERSION_PAGE_DATA_SIZE    30U         /* Page1 前 30 字节参与页和，后 2 字节存页和。 */
#define MAINBOARD_SW_VERSION_HEX_MARKER_TEXT   "MBSWVER1"  /* HEX 内固定版本标记，上位机通过它从固件镜像中提取大版本号。 */

static uint8_t s_mainboard_sw_version_record[MAINBOARD_SW_VERSION_RECORD_SIZE]; /* 最近一次构造或读取的 Page1 软件版本记录缓存。 */
static const char s_mainboard_sw_version_hex_marker[] = MAINBOARD_SW_VERSION_HEX_MARKER_TEXT MAINBOARD_SW_VERSION_BASE_TEXT; /* 把版本标记和版本号一起存入Flash，HEX文件和EEPROM记录都使用这里的版本号。 */

/*
 * 函数功能：按大端格式写入 32 位字段，保证地址、长度和 CRC 在 EEPROM 中直接可读。
 * 输入参数：data 为写入位置，value 为待写入 32 位值。
 * 返回参数：无。
 */
static void SwVersion_WriteBe32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value >> 24); /* 写入最高字节，方便上位机按地址原值显示。 */
  data[1] = (uint8_t)(value >> 16); /* 写入次高字节，保持多字节字段顺序一致。 */
  data[2] = (uint8_t)(value >> 8);  /* 写入次低字节。 */
  data[3] = (uint8_t)(value & 0xFFU); /* 写入最低字节。 */
}

/*
 * 函数功能：按大端格式读取 32 位字段，用于校验 Page1 记录范围是否属于当前 256KB 代码区。
 * 输入参数：data 为待读取字段起始地址。
 * 返回参数：读取出的 32 位值。
 */
static uint32_t SwVersion_ReadBe32(const uint8_t *data)
{
  return (((uint32_t)data[0] << 24) |
          ((uint32_t)data[1] << 16) |
          ((uint32_t)data[2] << 8) |
          (uint32_t)data[3]); /* 组合出的数值与 EEPROM 中大端字段一一对应。 */
}

/*
 * 函数功能：计算主控板 AT24C32 Page1 前 30 字节的 16 位累加和。
 * 输入参数：record 为 32 字节 Page1 缓存。
 * 返回参数：前 30 字节累加得到的页校验和。
 */
static uint16_t SwVersion_PageSum(const uint8_t *record)
{
  uint16_t sum = 0U; /* 16 位累加自然截断，保持和手查 EEPROM 页和规则一致。 */
  uint8_t index; /* 遍历 Page1 前 30 字节。 */

  for (index = 0U; index < MAINBOARD_SW_VERSION_PAGE_DATA_SIZE; ++index)
  {
    sum = (uint16_t)(sum + record[index]); /* 每个版本记录字节都参与页和，用于发现 EEPROM 单字节丢失。 */
  }

  return sum;
}

/*
 * 函数功能：刷新 Page1 记录末尾 2 字节页和。
 * 输入参数：record 为 32 字节 Page1 缓存。
 * 返回参数：无。
 */
static void SwVersion_FillPageSum(uint8_t *record)
{
  uint16_t sum = SwVersion_PageSum(record); /* 先按前 30 字节重新计算页和。 */

  record[30] = (uint8_t)(sum >> 8); /* 页和高字节放 byte30。 */
  record[31] = (uint8_t)(sum & 0xFFU); /* 页和低字节放 byte31。 */
}

/*
 * 函数功能：计算固定 256KB 产品代码区 CRC32。
 * 输入参数：无。
 * 返回参数：标准 CRC32 结果，初值和异或值均为 0xFFFFFFFF。
 */
static uint32_t SwVersion_CodeCrc32(void)
{
  const uint8_t *code = (const uint8_t *)(uintptr_t)MAINBOARD_SW_VERSION_CODE_START; /* 固定从 Flash 0x08000000 读取代码区。 */
  uint32_t crc = 0xFFFFFFFFUL; /* 标准 CRC32 初始值，上位机必须使用同一规则。 */
  uint32_t offset; /* 当前扫描的代码区偏移，范围固定为 0~0x3FFFF。 */
  uint8_t bit; /* 单字节 CRC32 的 8 次移位。 */

  for (offset = 0UL; offset < MAINBOARD_SW_VERSION_CODE_SIZE; ++offset)
  {
    crc ^= (uint32_t)code[offset]; /* 当前 Flash 字节进入 CRC 低 8 位。 */
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1); /* 反射 CRC32 多项式，和上位机 JS 计算保持一致。 */
    }
  }

  return crc ^ 0xFFFFFFFFUL; /* 标准 CRC32 输出前最终异或。 */
}

/*
 * 函数功能：构造当前固件应写入主控板 AT24C32 Page1 的软件版本记录。
 * 输入参数：record 为 32 字节输出缓存。
 * 返回参数：无。
 */
static void SwVersion_BuildRecord(uint8_t *record)
{
  uint8_t index; /* 拷贝固定大版本字符串时使用的字节下标。 */
  const char *version_text = &s_mainboard_sw_version_hex_marker[sizeof(MAINBOARD_SW_VERSION_HEX_MARKER_TEXT) - 1U]; /* Page1 大版本直接来自 HEX 标记后的字符串，避免上位机和固件使用两套来源。 */
  uint8_t version_len; /* 实际写入 EEPROM 的大版本字符串长度，不超过 8 字节。 */
  uint32_t crc32; /* 当前 256KB 代码区 CRC32。 */

  memset(record, 0, MAINBOARD_SW_VERSION_RECORD_SIZE); /* 先清零，保证保留字段和短版本字符串尾部稳定。 */
  record[0] = (uint8_t)MAINBOARD_SW_VERSION_MAGIC0; /* 第0字节写S，与后3字节组成固定标记SWV1。 */
  record[1] = (uint8_t)MAINBOARD_SW_VERSION_MAGIC1; /* 第1字节写W。 */
  record[2] = (uint8_t)MAINBOARD_SW_VERSION_MAGIC2; /* 第2字节写V。 */
  record[3] = (uint8_t)MAINBOARD_SW_VERSION_MAGIC3; /* 第3字节写1。 */
  record[4] = MAINBOARD_SW_VERSION_FORMAT; /* byte4 写入记录格式版本，后续扩字段时可区分。 */

  version_len = (strlen(version_text) > MAINBOARD_SW_VERSION_TEXT_SIZE) ?
                MAINBOARD_SW_VERSION_TEXT_SIZE :
                (uint8_t)strlen(version_text); /* 字符串末尾 0 不作为有效版本字符写入。 */

  for (index = 0U; index < version_len; ++index)
  {
    record[5U + index] = (uint8_t)version_text[index]; /* byte5~12 写入 V1.0/V1.1 等人工确定的大版本。 */
  }

  record[13] = MAINBOARD_SW_VERSION_CRC32_ALGO; /* byte13 写入 CRC32 算法编号，防止上位机按错误算法比较。 */
  SwVersion_WriteBe32(&record[14], MAINBOARD_SW_VERSION_CODE_START); /* byte14~17 写入固定校验起始地址 0x08000000。 */
  SwVersion_WriteBe32(&record[18], MAINBOARD_SW_VERSION_CODE_SIZE); /* byte18~21 写入固定校验长度 256KB。 */
  crc32 = SwVersion_CodeCrc32(); /* 计算当前代码区 CRC32，烧录文件变化会体现在这里。 */
  SwVersion_WriteBe32(&record[22], crc32); /* byte22~25 写入代码区 CRC32，用于和 HEX 解析结果比对。 */
  SwVersion_FillPageSum(record); /* byte30~31 写入 Page1 自身校验和，读取时先验证记录未损坏。 */
}

/*
 * 函数功能：校验主控板 AT24C32 Page1 软件版本记录是否符合当前格式。
 * 输入参数：record 为 32 字节 Page1 缓存。
 * 返回参数：MainboardSoftwareVersionStatus_t 状态码。
 */
static MainboardSoftwareVersionStatus_t SwVersion_ValidateRecord(const uint8_t *record)
{
  uint16_t stored_sum; /* Page1 byte30~31 存储的页和。 */
  uint16_t calc_sum; /* 根据 Page1 前 30 字节重新计算出的页和。 */

  if (record == 0)
  {
    return MAINBOARD_SW_VERSION_STATUS_BAD_PARAM; /* 空指针说明调用方没有提供 Page1 缓存。 */
  }

  stored_sum = (uint16_t)(((uint16_t)record[30] << 8) | record[31]); /* 页和按大端保存在记录末尾。 */
  calc_sum = SwVersion_PageSum(record); /* 重新计算页和，用于发现 EEPROM 记录损坏。 */
  if (stored_sum != calc_sum)
  {
    return MAINBOARD_SW_VERSION_STATUS_PAGE_SUM_FAIL; /* 页和失败时不再信任后续字段。 */
  }

  /* 开头4字节必须是SWV1，否则不能继续按软件版本记录读取。 */
  if ((record[0] != (uint8_t)MAINBOARD_SW_VERSION_MAGIC0) ||
      (record[1] != (uint8_t)MAINBOARD_SW_VERSION_MAGIC1) ||
      (record[2] != (uint8_t)MAINBOARD_SW_VERSION_MAGIC2) ||
      (record[3] != (uint8_t)MAINBOARD_SW_VERSION_MAGIC3))
  {
    return MAINBOARD_SW_VERSION_STATUS_MAGIC_FAIL; /* 固定标记不符，报告记录类型错误。 */
  }

  /* 格式号、CRC 算法或代码区范围任一不匹配时，上位机不能用该记录比较当前固件。 */
  if ((record[4] != MAINBOARD_SW_VERSION_FORMAT) ||
      (record[13] != MAINBOARD_SW_VERSION_CRC32_ALGO) ||
      (SwVersion_ReadBe32(&record[14]) != MAINBOARD_SW_VERSION_CODE_START) ||
      (SwVersion_ReadBe32(&record[18]) != MAINBOARD_SW_VERSION_CODE_SIZE))
  {
    return MAINBOARD_SW_VERSION_STATUS_FORMAT_FAIL; /* 格式、算法或 256KB 范围不匹配时，上位机不能拿来直接比较。 */
  }

  return MAINBOARD_SW_VERSION_STATUS_OK; /* 记录结构有效，CRC32 字段可用于和本地 HEX 计算结果比较。 */
}

/*
 * 函数功能：启动后自动同步主控板 AT24C32 Page1 软件版本记录。
 * 输入参数：无。
 * 返回参数：MainboardSoftwareVersionStatus_t 状态码，0 表示 EEPROM 已经保持当前固件版本。
 */
uint8_t MainboardSoftwareVersion_Sync(void)
{
  uint8_t current_record[MAINBOARD_SW_VERSION_RECORD_SIZE]; /* 当前固件计算出的目标 Page1 记录。 */
  uint8_t stored_record[MAINBOARD_SW_VERSION_RECORD_SIZE]; /* 从主控板 AT24C32 读出的旧 Page1 记录。 */

  SwVersion_BuildRecord(current_record); /* 每次启动都按当前 Flash 256KB 内容重新生成目标记录。 */
  memcpy(s_mainboard_sw_version_record, current_record, MAINBOARD_SW_VERSION_RECORD_SIZE); /* 缓存当前目标记录，便于调试读取最近计算值。 */

  if (IIC_AT24C32_ReadBytes(MAINBOARD_SW_VERSION_PAGE1_ADDR, stored_record, MAINBOARD_SW_VERSION_RECORD_SIZE) != 0U) /* 只有 Page1 读取成功，才能比较旧记录并决定是否免写。 */
  {
    if ((SwVersion_ValidateRecord(stored_record) == MAINBOARD_SW_VERSION_STATUS_OK) &&
        (memcmp(stored_record, current_record, MAINBOARD_SW_VERSION_RECORD_SIZE) == 0))
    {
      return MAINBOARD_SW_VERSION_STATUS_OK; /* EEPROM 已经存着当前固件记录，不重复写，减少 AT24C32 磨损。 */
    }
  }

  if (IIC_AT24C32_WriteBytes(MAINBOARD_SW_VERSION_PAGE1_ADDR, current_record, MAINBOARD_SW_VERSION_RECORD_SIZE) == 0U)
  {
    return MAINBOARD_SW_VERSION_STATUS_EEPROM_WRITE_FAIL; /* 写入未应答时保持主流程继续运行，但向外部通信暴露失败状态。 */
  }

  if (IIC_AT24C32_ReadBytes(MAINBOARD_SW_VERSION_PAGE1_ADDR, stored_record, MAINBOARD_SW_VERSION_RECORD_SIZE) == 0U)
  {
    return MAINBOARD_SW_VERSION_STATUS_EEPROM_WRITE_FAIL; /* 写后无法读回，不能确认EEPROM已经保存新记录。 */
  }

  if (memcmp(stored_record, current_record, MAINBOARD_SW_VERSION_RECORD_SIZE) != 0)
  {
    return MAINBOARD_SW_VERSION_STATUS_EEPROM_WRITE_FAIL; /* 读回和目标记录不同，说明写入过程不可靠。 */
  }

  return (uint8_t)SwVersion_ValidateRecord(stored_record); /* 最后再跑格式和页和校验，保证写入记录可被上位机解析。 */
}

/*
 * 函数功能：先更新EEPROM版本记录，再读出Page1，拼成发给上位机的版本数据。
 * 输入参数：payload 为输出缓存，payload_size 为缓存大小，payload_len 返回实际载荷长度。
 * 返回参数：1 表示已生成载荷，0 表示参数非法。
 */
uint8_t MainboardSoftwareVersion_ReadPayload(uint8_t *payload, uint16_t payload_size, uint16_t *payload_len)
{
  MainboardSoftwareVersionStatus_t sync_status; /* 外部读取前主动同步 EEPROM 的结果，用于区分写入失败和记录内容损坏。 */
  MainboardSoftwareVersionStatus_t status; /* Page1 记录校验状态，会放在载荷 byte0 给上位机显示。 */

  if ((payload == 0) || (payload_len == 0) || (payload_size < MAINBOARD_SW_VERSION_PAYLOAD_SIZE))
  {
    return 0U; /* 输出缓存不足时拒绝组包，避免外部通信发送截断版本记录。 */
  }

  sync_status = (MainboardSoftwareVersionStatus_t)MainboardSoftwareVersion_Sync(); /* 读取前先写入当前固件记录，避免烧录后 Page1 仍停留在旧版本。 */
  memset(payload, 0, MAINBOARD_SW_VERSION_PAYLOAD_SIZE); /* 先清零，EEPROM 读取失败时也有确定的原始记录区。 */
  if (IIC_AT24C32_ReadBytes(MAINBOARD_SW_VERSION_PAGE1_ADDR,
                            s_mainboard_sw_version_record,
                            MAINBOARD_SW_VERSION_RECORD_SIZE) == 0U)
  {
    payload[0] = (uint8_t)MAINBOARD_SW_VERSION_STATUS_EEPROM_READ_FAIL; /* EEPROM 不应答时上报明确状态。 */
    *payload_len = MAINBOARD_SW_VERSION_PAYLOAD_SIZE; /* 载荷长度仍固定，便于上位机统一解析。 */
    return 1U;
  }

  status = SwVersion_ValidateRecord(s_mainboard_sw_version_record); /* 读取成功后校验 Page1 自身结构。 */
  if ((sync_status == MAINBOARD_SW_VERSION_STATUS_EEPROM_READ_FAIL) ||
      (sync_status == MAINBOARD_SW_VERSION_STATUS_EEPROM_WRITE_FAIL))
  {
    payload[0] = (uint8_t)sync_status; /* 同步阶段读写失败优先上报，防止现场只看到后续旧记录的 Page1 校验错误。 */
  }
  else
  {
    payload[0] = (uint8_t)status; /* byte0 是状态码，0 表示后续 32 字节记录有效。 */
  }
  memcpy(&payload[1], s_mainboard_sw_version_record, MAINBOARD_SW_VERSION_RECORD_SIZE); /* byte1~32 是 AT24C32 Page1 原始记录。 */
  *payload_len = MAINBOARD_SW_VERSION_PAYLOAD_SIZE; /* 当前协议固定 33 字节。 */
  return 1U;
}
