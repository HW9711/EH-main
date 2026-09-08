#ifndef __EXTERNAL_COMM_PROTOCOL_H
#define __EXTERNAL_COMM_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXTERNAL_COMM_MAX_FRAME_SIZE       150U   /* 正式协议整帧最大字节数；调大时必须同步检查 UART2 接收缓存和上位机帧长限制。 */
#define EXTERNAL_COMM_FRAME_FIXED_SIZE     16U    /* 帧头4+传输码1+长度2+功能码1+区域码1+信息码1+CRC2+帧尾4。 */
#define EXTERNAL_COMM_MAX_INFO_SIZE        (EXTERNAL_COMM_MAX_FRAME_SIZE - EXTERNAL_COMM_FRAME_FIXED_SIZE) /* InforArea 最大可用字节数。 */

/* 传输方向码：0x01 为本机上传，0x02 为外部设备下发。 */
#define EXTERNAL_COMM_TRAN_UPLOAD          0x01U  /* 本机主动上传或应答外部设备。 */
#define EXTERNAL_COMM_TRAN_DOWNLOAD        0x02U  /* 外部设备下发控制、设置、读写 EEPROM 命令。 */

/* 上行功能码必须与上位机约定一致；修改数值会改变上位机对报文的识别。 */
#define EXTERNAL_COMM_FUNC_EEPROM_UPLOAD   0x01U  /* 上传 EEPROM 单页有效数据。 */
#define EXTERNAL_COMM_FUNC_HOST_SETTING    0x02U  /* 预留：上传主机设置内容。 */
#define EXTERNAL_COMM_FUNC_HOST_RUNNING    0x03U  /* 上传主机运行内容，目前用于报警和电机状态。 */
#define EXTERNAL_COMM_FUNC_SOFTWARE_VERSION 0x0BU /* 上传主控板 AT24C32 保存的软件版本记录。 */
#define EXTERNAL_COMM_FUNC_HEARTBEAT       0xAAU  /* 周期上传外部通信心跳。 */
#define EXTERNAL_COMM_FUNC_HOST_EXIT       0xBBU  /* 预留的主机主动退出上行功能码；当前退出通知使用 0xDD 应答携带 0xBB。 */
#define EXTERNAL_COMM_FUNC_PLUG_SWITCH     0xCCU  /* 预留：上传接插切换内容。 */
#define EXTERNAL_COMM_FUNC_ACK             0xDDU  /* 上传命令执行结果应答。 */
#define EXTERNAL_COMM_FUNC_HOST_CONTROL    0xEEU  /* 预留：上传主机控制内容。 */

/* 下行 0x0D 控制电机状态定时上传：数据区 0 关闭、1 开启；上传周期 50ms，不改变 100ms 心跳和电机控制周期。 */
#define EXTERNAL_COMM_FUNC_MOTOR_TELEMETRY_SUBSCRIBE 0x0DU

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
    EXTERNAL_COMM_BUILD_BAD_PARAM,       /* 输出缓存、长度指针或所需的数据指针为空。 */
    EXTERNAL_COMM_BUILD_OVERFLOW         /* 输出缓存不足，或 InforArea 字节数超过协议上限。 */
} ExternalCommBuildResult_t;

typedef struct
{
    uint8_t tran_code;                  /* 传输方向：0x01 为主控上传，0x02 为上位机下发。 */
    uint16_t length;                    /* Length 字段，包含帧头到帧尾的整帧字节数。 */
    uint8_t fun_code;                   /* 功能码，决定设置参数、控制输出或访问 EEPROM。 */
    uint8_t area_code;                  /* 区域码，具体表示通道、参数编号或页号，由功能码决定。 */
    uint8_t info_code;                  /* 信息码，下行命令要求为 0xFF，上行可表示应答结果或状态类型。 */
    uint16_t info_len;                  /* InforArea 数据字节数，不包含固定字段、CRC 和帧尾。 */
    uint8_t info_area[EXTERNAL_COMM_MAX_INFO_SIZE]; /* 命令数据区，只保存 info_len 个有效字节。 */
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

/* 构造 0xAA 心跳帧；调用方按设备在线和运行状态决定要发送哪些字段。 */
ExternalCommBuildResult_t ExternalCommProtocol_BuildHeartbeat(const uint8_t *heartbeat_info,
                                                              uint16_t heartbeat_info_len,
                                                              uint8_t *out_buf,
                                                              uint16_t out_size,
                                                              uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_COMM_PROTOCOL_H */
