#include "external_comm_protocol.h"

#include "common.h"

#include <string.h>

/* 固定 4 字节帧头，对应协议文档中的 D7 CA F8 F1。 */
static const uint8_t s_external_comm_head[4] = {0xD7U, 0xCAU, 0xF8U, 0xF1U};
/* 固定 4 字节帧尾，对应协议文档中的 BF C6 BC C4。 */
static const uint8_t s_external_comm_tail[4] = {0xBFU, 0xC6U, 0xBCU, 0xC4U};

/*
 * 函数功能：把高字节在前的两个字节合成 16 位数值，例如 0x0B 0xB8 得到 3000。
 * 输入参数：data 指向至少 2 字节的数据。
 * 返回参数：合成后的 16 位数值。
 */
static uint16_t ExternalComm_ReadBE16(const uint8_t *data)
{
    /* data[0] 是高字节，左移 8 位后与低字节合成 16 位数值。 */
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/*
 * 函数功能：把 16 位数值拆成两个字节，高字节写在前面。
 * 输入参数：data 指向至少 2 字节的输出空间；value 为待写入数值。
 * 返回参数：无。
 */
static void ExternalComm_WriteBE16(uint8_t *data, uint16_t value)
{
    /* 先写高字节，保证 Length 和 CRC 在串口线上按协议顺序发送。 */
    data[0] = (uint8_t)(value >> 8);
    /* 再写低字节，屏蔽高位避免隐式截断带来编译器告警。 */
    data[1] = (uint8_t)(value & 0xFFU);
}

/*
 * 函数功能：逐字节比较接收内容与固定帧头或帧尾。
 * 输入参数：data 为待检查数据；pattern 为固定字节；len 为比较字节数，两处数据都必须足够长。
 * 返回参数：全部相同返回 1，任一字节不同返回 0。
 */
static uint8_t ExternalComm_MatchBytes(const uint8_t *data, const uint8_t *pattern, uint16_t len)
{
    /* i 逐字节扫描，用于比较帧头和帧尾固定字节。 */
    uint16_t i;

    /* 只比较调用方指定的 len 字节，当前帧头和帧尾都使用 4 字节。 */
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

/*
 * 函数功能：在接收数据中找到帧头，检查整帧长度、帧尾和 CRC，再取出命令字段。
 * 输入参数：data 为接收数据；data_len 为字节数；frame 接收解析结果。
 * 返回参数：成功返回 EXTERNAL_COMM_PARSE_OK；失败返回帧未收全、长度错误、帧尾错误等原因。
 */
ExternalCommParseResult_t ExternalCommProtocol_Parse(const uint8_t *data,
                                                     uint16_t data_len,
                                                     ExternalCommFrame_t *frame)
{
    /* start 是当前检查的帧头位置，从缓存开头逐字节向后查找。 */
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

        /* 已找到帧头，但剩余数据不足最短 16 字节，通知调用方等待后续数据。 */
        if ((uint16_t)(data_len - start) < EXTERNAL_COMM_FRAME_FIXED_SIZE)
        {
            return EXTERNAL_COMM_PARSE_INCOMPLETE;
        }

        /* 记住帧头位置，后面按协议中的字节偏移读取字段。 */
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
        /* CRC 从 TranCode 算到 InforArea 末尾，包含 Length、FunCode、AreaCode 和 InforCode，不包含帧头、CRC 和帧尾。 */
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
        /* 保存 InforArea 的字节数，命令处理函数据此检查参数是否收全。 */
        frame->info_len = info_len;
        /* InforArea 非空时才复制，空数据区不需要调用 memcpy。 */
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

/*
 * 函数功能：按正式协议生成完整发送帧，Length 为整帧字节数，CRC 高字节在前。
 * 输入参数：tran_code/fun_code/area_code/info_code 为协议字段；info_area/info_len 为数据及字节数；out_buf/out_size 为输出缓存及容量；out_len 接收帧长。
 * 返回参数：成功返回 EXTERNAL_COMM_BUILD_OK；指针错误或缓存不足返回对应错误码。
 */
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

    /* 输出缓存和长度指针不能为空；info_len 非零时，输入数据指针也不能为空。 */
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

    /* InforArea 非空时，从帧偏移 10 开始复制命令数据。 */
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

/*
 * 函数功能：生成 0xDD 命令应答帧，把执行结果码写入 InforCode。
 * 输入参数：ack_code 为结果码；ack_info/ack_info_len 为应答数据及字节数；out_buf/out_size 为输出缓存及容量；out_len 接收帧长。
 * 返回参数：成功返回 EXTERNAL_COMM_BUILD_OK，否则返回指针错误或空间不足的错误码。
 */
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

/*
 * 函数功能：生成 0xAA 心跳帧，发送调用方已经整理好的在线和运行状态。
 * 输入参数：heartbeat_info/heartbeat_info_len 为心跳数据及字节数；out_buf/out_size 为输出缓存及容量；out_len 接收帧长。
 * 返回参数：成功返回 EXTERNAL_COMM_BUILD_OK；心跳数据为空、指针错误或空间不足时返回错误码。
 */
ExternalCommBuildResult_t ExternalCommProtocol_BuildHeartbeat(const uint8_t *heartbeat_info,
                                                              uint16_t heartbeat_info_len,
                                                              uint8_t *out_buf,
                                                              uint16_t out_size,
                                                              uint16_t *out_len)
{
    /* 心跳至少要包含在线状态；调用方没有提供数据时不生成空心跳帧。 */
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
