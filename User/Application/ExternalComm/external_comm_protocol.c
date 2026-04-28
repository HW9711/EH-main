#include "external_comm_protocol.h"

#include "common.h"

#include <string.h>

/* 固定 4 字节帧头，对应协议文档中的 D7 CA F8 F1。 */
static const uint8_t s_external_comm_head[4] = {0xD7U, 0xCAU, 0xF8U, 0xF1U};
/* 固定 4 字节帧尾，对应协议文档中的 BF C6 BC C4。 */
static const uint8_t s_external_comm_tail[4] = {0xBFU, 0xC6U, 0xBCU, 0xC4U};

/* 协议里的 16 位字段均按高字节在前处理，和示例 0x0B 0xB8 = 3000 保持一致。 */
static uint16_t ExternalComm_ReadBE16(const uint8_t *data)
{
    /* data[0] 是高字节，左移 8 位后与低字节合成 16 位数值。 */
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void ExternalComm_WriteBE16(uint8_t *data, uint16_t value)
{
    /* 先写高字节，保证 Length 和 CRC 在串口线上按协议顺序发送。 */
    data[0] = (uint8_t)(value >> 8);
    /* 再写低字节，屏蔽高位避免隐式截断带来编译器告警。 */
    data[1] = (uint8_t)(value & 0xFFU);
}

static uint8_t ExternalComm_MatchBytes(const uint8_t *data, const uint8_t *pattern, uint16_t len)
{
    /* i 逐字节扫描，用于比较帧头和帧尾固定字节。 */
    uint16_t i;

    /* len 由调用方传入，当前只用于 4 字节帧头/帧尾，也保留通用性。 */
    for (i = 0U; i < len; ++i)
    {
        /* 任意一个字节不同就认为匹配失败，解析器继续找下一个帧头。 */
        if (data[i] != pattern[i])
        {
            return 0U;
        }
    }

    /* 全部字节一致，返回 1 表示匹配成功。 */
    return 1U;
}

ExternalCommParseResult_t ExternalCommProtocol_Parse(const uint8_t *data,
                                                     uint16_t data_len,
                                                     ExternalCommFrame_t *frame)
{
    /* start 是接收缓存中的候选帧头偏移。 */
    uint16_t start;
    /* frame_len 来自协议 Length 字段，V1 按整帧长度处理。 */
    uint16_t frame_len;
    /* info_len 是 InforArea 字节数，由整帧长度减固定字段得出。 */
    uint16_t info_len;
    /* crc_pos 是 CRC16_H 在帧内的偏移。 */
    uint16_t crc_pos;
    /* calc_crc 保存本机按 TranCode..InforArea 重新计算的 CRC。 */
    uint16_t calc_crc;
    /* recv_crc 保存帧内收到的 CRC16。 */
    uint16_t recv_crc;
    /* frame_buf 指向已经定位到帧头的缓存位置，避免反复叠加 start。 */
    const uint8_t *frame_buf;

    /* 入参为空时无法安全解析，返回未找到，调用层直接丢弃本次缓存。 */
    if ((data == NULL) || (frame == NULL))
    {
        return EXTERNAL_COMM_PARSE_NOT_FOUND;
    }

    /* 小于最短帧 16 字节时，说明 DMA 空闲包还不足一帧。 */
    if (data_len < EXTERNAL_COMM_FRAME_FIXED_SIZE)
    {
        return EXTERNAL_COMM_PARSE_INCOMPLETE;
    }

    /* DMA 空闲包可能带有前导噪声，这里先定位帧头再按 Length 校验整帧。 */
    for (start = 0U; start <= (uint16_t)(data_len - 4U); ++start)
    {
        /* 当前偏移不是帧头就跳过，继续在缓存中向后找。 */
        if (ExternalComm_MatchBytes(&data[start], s_external_comm_head, 4U) == 0U)
        {
            continue;
        }

        /* 找到帧头后，剩余字节仍不足固定帧长，保留“不完整”语义。 */
        if ((uint16_t)(data_len - start) < EXTERNAL_COMM_FRAME_FIXED_SIZE)
        {
            return EXTERNAL_COMM_PARSE_INCOMPLETE;
        }

        /* 将候选帧起点固化，后续字段偏移均相对帧头计算。 */
        frame_buf = &data[start];
        /* Length_H/Length_L 位于帧偏移 5/6，按大端读出。 */
        frame_len = ExternalComm_ReadBE16(&frame_buf[5]);
        /* 长度过短或超过 UART2 单包上限都视为非法长度。 */
        if ((frame_len < EXTERNAL_COMM_FRAME_FIXED_SIZE) || (frame_len > EXTERNAL_COMM_MAX_FRAME_SIZE))
        {
            return EXTERNAL_COMM_PARSE_BAD_LENGTH;
        }

        /* 收到的数据少于 Length 声明时，不能继续取 CRC 和帧尾。 */
        if ((uint16_t)(data_len - start) < frame_len)
        {
            return EXTERNAL_COMM_PARSE_INCOMPLETE;
        }

        /* 帧尾必须落在 Length 指定的最后 4 字节。 */
        if (ExternalComm_MatchBytes(&frame_buf[frame_len - 4U], s_external_comm_tail, 4U) == 0U)
        {
            return EXTERNAL_COMM_PARSE_BAD_TAIL;
        }

        /* 固定帧字段共 16 字节，扣掉后就是 InforArea 长度。 */
        info_len = (uint16_t)(frame_len - EXTERNAL_COMM_FRAME_FIXED_SIZE);
        /* CRC 位于 InforArea 后面，固定字段到 InforArea 起点是 10 字节。 */
        crc_pos = (uint16_t)(10U + info_len);
        /* V1 占位规则：CRC 覆盖 TranCode、Length、FunCode、AreaCode、InforCode、InforArea。 */
        calc_crc = Common_Crc16((uint8_t *)&frame_buf[4], (uint16_t)(6U + info_len));
        /* 帧内 CRC 同样按高字节在前读取。 */
        recv_crc = ExternalComm_ReadBE16(&frame_buf[crc_pos]);
        /* CRC 不一致说明内容被破坏或上位机规则不同，本帧丢弃。 */
        if (calc_crc != recv_crc)
        {
            return EXTERNAL_COMM_PARSE_BAD_CRC;
        }

        /* 传输码字段，0x02 表示下行，调度层再判断方向。 */
        frame->tran_code = frame_buf[4];
        /* 保存整帧长度，便于后续调试或协议扩展。 */
        frame->length = frame_len;
        /* 功能码字段，决定设置、控制、EEPROM、权限等分支。 */
        frame->fun_code = frame_buf[7];
        /* 区域码字段，决定具体参数或 EEPROM 页区域。 */
        frame->area_code = frame_buf[8];
        /* 信息码字段，当前下行要求为 0xFF。 */
        frame->info_code = frame_buf[9];
        /* 保存载荷长度，业务层按长度校验参数宽度。 */
        frame->info_len = info_len;
        /* 有载荷时才复制，避免 0 长度 memcpy 在不同库实现下产生歧义。 */
        if (info_len > 0U)
        {
            /* 只复制 InforArea，不把 CRC 和帧尾带给业务层。 */
            memcpy(frame->info_area, &frame_buf[10], info_len);
        }

        /* 至此帧头、长度、帧尾、CRC 全部通过，返回解析成功。 */
        return EXTERNAL_COMM_PARSE_OK;
    }

    /* 扫完整个 DMA 缓存仍未找到帧头。 */
    return EXTERNAL_COMM_PARSE_NOT_FOUND;
}

ExternalCommBuildResult_t ExternalCommProtocol_BuildFrame(uint8_t tran_code,
                                                          uint8_t fun_code,
                                                          uint8_t area_code,
                                                          uint8_t info_code,
                                                          const uint8_t *info_area,
                                                          uint16_t info_len,
                                                          uint8_t *out_buf,
                                                          uint16_t out_size,
                                                          uint16_t *out_len)
{
    /* frame_len 是本次要输出的整帧长度。 */
    uint16_t frame_len;
    /* crc 保存对 TranCode..InforArea 计算出的 CRC16。 */
    uint16_t crc;
    /* crc_pos 是 CRC16_H 写入位置。 */
    uint16_t crc_pos;

    /* 输出缓存、输出长度指针必须有效；有载荷时载荷指针也必须有效。 */
    if ((out_buf == NULL) || (out_len == NULL) ||
        ((info_len > 0U) && (info_area == NULL)))
    {
        return EXTERNAL_COMM_BUILD_BAD_PARAM;
    }

    /* InforArea 不能超过 UART2 当前单帧上限扣除固定字段后的空间。 */
    if (info_len > EXTERNAL_COMM_MAX_INFO_SIZE)
    {
        return EXTERNAL_COMM_BUILD_OVERFLOW;
    }

    /* V1 Length 字段直接等于固定 16 字节加 InforArea 长度。 */
    frame_len = (uint16_t)(EXTERNAL_COMM_FRAME_FIXED_SIZE + info_len);
    /* 调用方提供的发送缓存不足时禁止写入，避免越界。 */
    if (out_size < frame_len)
    {
        return EXTERNAL_COMM_BUILD_OVERFLOW;
    }

    /* 写入固定帧头。 */
    memcpy(&out_buf[0], s_external_comm_head, 4U);
    /* 写入传输码：本任务上传时通常为 0x01。 */
    out_buf[4] = tran_code;
    /* 写入整帧长度，高字节在前。 */
    ExternalComm_WriteBE16(&out_buf[5], frame_len);
    /* 写入功能码。 */
    out_buf[7] = fun_code;
    /* 写入区域码。 */
    out_buf[8] = area_code;
    /* 写入信息码。 */
    out_buf[9] = info_code;

    /* 有 InforArea 时把业务载荷复制到固定字段之后。 */
    if (info_len > 0U)
    {
        memcpy(&out_buf[10], info_area, info_len);
    }

    /* CRC16 高字节先发，便于上位机按文档直接显示和比对。 */
    crc = Common_Crc16(&out_buf[4], (uint16_t)(6U + info_len));
    /* CRC 写在 InforArea 后面。 */
    crc_pos = (uint16_t)(10U + info_len);
    /* 按高字节在前写入 CRC。 */
    ExternalComm_WriteBE16(&out_buf[crc_pos], crc);
    /* CRC 后紧跟固定帧尾。 */
    memcpy(&out_buf[crc_pos + 2U], s_external_comm_tail, 4U);

    /* 输出最终帧长，调用方按这个长度调用 Uart2_SendPacket。 */
    *out_len = frame_len;
    /* 组帧完成。 */
    return EXTERNAL_COMM_BUILD_OK;
}

ExternalCommBuildResult_t ExternalCommProtocol_BuildAck(uint8_t ack_code,
                                                        const uint8_t *ack_info,
                                                        uint16_t ack_info_len,
                                                        uint8_t *out_buf,
                                                        uint16_t out_size,
                                                        uint16_t *out_len)
{
    /* 0xDD 应答统一使用上传传输码、无区域码，ack_code 放到 InforCode。 */
    return ExternalCommProtocol_BuildFrame(EXTERNAL_COMM_TRAN_UPLOAD,
                                           EXTERNAL_COMM_FUNC_ACK,
                                           EXTERNAL_COMM_AREA_NONE,
                                           ack_code,
                                           ack_info,
                                           ack_info_len,
                                           out_buf,
                                           out_size,
                                           out_len);
}

ExternalCommBuildResult_t ExternalCommProtocol_BuildHeartbeat(const uint8_t *heartbeat_info,
                                                              uint16_t heartbeat_info_len,
                                                              uint8_t *out_buf,
                                                              uint16_t out_size,
                                                              uint16_t *out_len)
{
    /* 心跳至少要带在线状态字段，空载荷说明调用方没有完成状态采集。 */
    if ((heartbeat_info == NULL) || (heartbeat_info_len == 0U))
    {
        /* 返回参数错误，避免 UART2 发出字段缺失的 0xAA 帧。 */
        return EXTERNAL_COMM_BUILD_BAD_PARAM;
    }

    /* 心跳固定 FunCode=0xAA，AreaCode/InforCode 都填 0xFF。 */
    return ExternalCommProtocol_BuildFrame(EXTERNAL_COMM_TRAN_UPLOAD,
                                           EXTERNAL_COMM_FUNC_HEARTBEAT,
                                           EXTERNAL_COMM_AREA_NONE,
                                           EXTERNAL_COMM_INFO_NONE,
                                           heartbeat_info,
                                           heartbeat_info_len,
                                           out_buf,
                                           out_size,
                                           out_len);
}
