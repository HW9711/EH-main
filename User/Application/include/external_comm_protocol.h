#ifndef __EXTERNAL_COMM_PROTOCOL_H
#define __EXTERNAL_COMM_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXTERNAL_COMM_MAX_FRAME_SIZE       150U   /* UART2 当前单包缓存上限，禁止构造超过此长度的应用帧。 */
#define EXTERNAL_COMM_FRAME_FIXED_SIZE     16U    /* 帧头4+传输码1+长度2+功能码1+区域码1+信息码1+CRC2+帧尾4。 */
#define EXTERNAL_COMM_MAX_INFO_SIZE        (EXTERNAL_COMM_MAX_FRAME_SIZE - EXTERNAL_COMM_FRAME_FIXED_SIZE) /* InforArea 最大可用字节数。 */

/* 传输方向码：0x01 为本机上传，0x02 为外部设备下发。 */
#define EXTERNAL_COMM_TRAN_UPLOAD          0x01U  /* 本机主动上传或应答外部设备。 */
#define EXTERNAL_COMM_TRAN_DOWNLOAD        0x02U  /* 外部设备下发控制、设置、读写 EEPROM 命令。 */

/* 应用层功能码，协议字段保持独立，便于后续按文档调整映射。 */
#define EXTERNAL_COMM_FUNC_EEPROM_UPLOAD   0x01U  /* 上传 EEPROM 单页有效数据。 */
#define EXTERNAL_COMM_FUNC_HOST_SETTING    0x02U  /* 预留：上传主机设置内容。 */
#define EXTERNAL_COMM_FUNC_HOST_RUNNING    0x03U  /* 预留：上传主机运行内容。 */
#define EXTERNAL_COMM_FUNC_SOFTWARE_VERSION 0x0BU /* 上传主控板 AT24C32 保存的软件版本记录。 */
#define EXTERNAL_COMM_FUNC_HEARTBEAT       0xAAU  /* 周期上传外部通信心跳。 */
#define EXTERNAL_COMM_FUNC_HOST_EXIT       0xBBU  /* 预留：上传主机主动退出外部控制。 */
#define EXTERNAL_COMM_FUNC_PLUG_SWITCH     0xCCU  /* 预留：上传接插切换内容。 */
#define EXTERNAL_COMM_FUNC_ACK             0xDDU  /* 上传命令执行结果应答。 */
#define EXTERNAL_COMM_FUNC_HOST_CONTROL    0xEEU  /* 预留：上传主机控制内容。 */

#define EXTERNAL_COMM_AREA_NONE            0xFFU  /* 无区域码时填充 0xFF。 */
#define EXTERNAL_COMM_INFO_NONE            0xFFU  /* 无信息码时填充 0xFF。 */

#define EXTERNAL_COMM_ACK_RUN_SET_OK       0x01U  /* 运行值设置成功。 */
#define EXTERNAL_COMM_ACK_RUN_SET_FAILED   0x02U  /* 运行值设置失败。 */
#define EXTERNAL_COMM_ACK_CONTROL_OK       0x03U  /* 控制命令执行成功。 */
#define EXTERNAL_COMM_ACK_CONTROL_FAILED   0x04U  /* 控制命令执行失败。 */
#define EXTERNAL_COMM_ACK_MEMORY_OK        0x05U  /* EEPROM 读写相关命令成功。 */
#define EXTERNAL_COMM_ACK_MEMORY_FAILED    0x06U  /* EEPROM 读写相关命令失败。 */
#define EXTERNAL_COMM_ACK_EXTERNAL_OK      0xAAU  /* 外部控制申请成功。 */
#define EXTERNAL_COMM_ACK_EXTERNAL_FAILED  0xABU  /* 外部控制申请失败。 */
#define EXTERNAL_COMM_ACK_AUTH_FAILED      0xBBU  /* 注册码或权限码校验失败。 */
#define EXTERNAL_COMM_ACK_PERMISSION_OK    0xFEU  /* 功能权限开放成功。 */
#define EXTERNAL_COMM_ACK_PERMISSION_FAIL  0xFFU  /* 功能权限未开放。 */

typedef enum
{
    EXTERNAL_COMM_PARSE_OK = 0,          /* 已找到完整合法帧。 */
    EXTERNAL_COMM_PARSE_NOT_FOUND,       /* 接收缓存里未找到帧头。 */
    EXTERNAL_COMM_PARSE_INCOMPLETE,      /* 找到帧头但缓存长度不足一整帧。 */
    EXTERNAL_COMM_PARSE_BAD_LENGTH,      /* Length 字段小于最短帧或超过本机上限。 */
    EXTERNAL_COMM_PARSE_BAD_TAIL,        /* Length 指向的位置不是固定帧尾。 */
    EXTERNAL_COMM_PARSE_BAD_CRC          /* CRC16 校验不通过。 */
} ExternalCommParseResult_t;

typedef enum
{
    EXTERNAL_COMM_BUILD_OK = 0,          /* 组帧成功。 */
    EXTERNAL_COMM_BUILD_BAD_PARAM,       /* 输出缓存、长度指针或载荷指针非法。 */
    EXTERNAL_COMM_BUILD_OVERFLOW         /* 输出缓存不足或载荷超过协议上限。 */
} ExternalCommBuildResult_t;

typedef struct
{
    /* 解析后的协议字段，info_area 只保存 InforArea 有效载荷。 */
    uint8_t tran_code;
    uint16_t length;
    uint8_t fun_code;
    uint8_t area_code;
    uint8_t info_code;
    uint16_t info_len;
    uint8_t info_area[EXTERNAL_COMM_MAX_INFO_SIZE];
} ExternalCommFrame_t;

/* 从 UART2 一次接收缓存中查找并校验完整帧，CRC 覆盖 TranCode 到 InforArea。 */
ExternalCommParseResult_t ExternalCommProtocol_Parse(const uint8_t *data,
                                                     uint16_t data_len,
                                                     ExternalCommFrame_t *frame);

/* 按当前 V1 规则组帧：Length 为整帧长度，CRC16 高字节在前。 */
ExternalCommBuildResult_t ExternalCommProtocol_BuildFrame(uint8_t tran_code,
                                                          uint8_t fun_code,
                                                          uint8_t area_code,
                                                          uint8_t info_code,
                                                          const uint8_t *info_area,
                                                          uint16_t info_len,
                                                          uint8_t *out_buf,
                                                          uint16_t out_size,
                                                          uint16_t *out_len);

/* 构造 0xDD 应答帧，ack_code 写入 InforCode。 */
ExternalCommBuildResult_t ExternalCommProtocol_BuildAck(uint8_t ack_code,
                                                        const uint8_t *ack_info,
                                                        uint16_t ack_info_len,
                                                        uint8_t *out_buf,
                                                        uint16_t out_size,
                                                        uint16_t *out_len);

/* 构造 0xAA 心跳帧，心跳载荷按协议在线/运行状态动态增减。 */
ExternalCommBuildResult_t ExternalCommProtocol_BuildHeartbeat(const uint8_t *heartbeat_info,
                                                              uint16_t heartbeat_info_len,
                                                              uint8_t *out_buf,
                                                              uint16_t out_size,
                                                              uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_PROTOCOL_H */
