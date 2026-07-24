#ifndef __BOARD_PROFILE_H
#define __BOARD_PROFILE_H

#include <stdint.h>
#include "board.h"

/*
 * Board profile shell.
 * Keep board.h as current source of truth during migration.
 */

#define BOARD_PROFILE_HAS_K1K2            BOARD_HAS_K1K2
#define RFID_USE_DUAL_UART_MODE           0U  /* RFID硬件模式：1表示逻辑A固定UART3、逻辑B固定UART9；0表示UART3+R200-K8选通。 */

/*
 * RFID 请求/应答与确认掉线统计开关：
 * 1U：主控分别累计 A/B 通道的读取命令、有效回包、无有效回包、异常帧、在线监测完成和确认掉线，并在心跳尾部上报。
 * 0U：不保留统计状态，也不追加心跳统计扩展，心跳字节格式恢复为原版本。
 */
#ifndef RFID_LINK_STATS_ENABLE
#define RFID_LINK_STATS_ENABLE             1U
#endif

#if ((RFID_LINK_STATS_ENABLE != 0U) && (RFID_LINK_STATS_ENABLE != 1U))
#error "RFID_LINK_STATS_ENABLE must be 0U or 1U"
#endif

/*
 * UART2 简易外控协议开关：
 * 1U：识别并执行 AA BB CC FunCode EE FF 固定 6 字节指令，同时保留原外控协议。
 * 0U：完全屏蔽简易协议识别和执行，UART2 只运行原外控协议。
 */
#ifndef EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE
#define EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE 1U
#endif

#if ((EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE != 0U) && (EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE != 1U))
#error "EXTERNAL_COMM_SIMPLE_PROTOCOL_ENABLE must be 0U or 1U"
#endif

/*
 * R200-K8旧硬件的RFID通道交换开关，仅在RFID_USE_DUAL_UART_MODE=0U时生效：
 * 1U：逻辑A选通R200-K8物理B，逻辑B选通物理A，用于旧硬件A/B线束交叉安装。
 * 0U：逻辑A选通物理A，逻辑B选通物理B，恢复旧硬件原始通道顺序。
 * 双串口硬件不读取该宏，始终保持逻辑A=UART3、逻辑B=UART9。
 */
#ifndef RFID_R200_AB_SWAP_ENABLE
#define RFID_R200_AB_SWAP_ENABLE          0U
#endif

#if ((RFID_R200_AB_SWAP_ENABLE != 0U) && (RFID_R200_AB_SWAP_ENABLE != 1U))
#error "RFID_R200_AB_SWAP_ENABLE must be 0U or 1U"
#endif

/*
 * 手柄物理接口交换开关：
 * 1U：逻辑A使用原物理B接口，逻辑B使用原物理A接口，用于当前镜像安装线束。
 * 0U：逻辑A/B分别使用原物理A/B接口，可直接恢复旧硬件接线方式。
 * 该配置只交换短接、实体键、EEPROM和电机等手柄接口资源，不交换固定线束的RFID串口。
 * R200-K8旧硬件的RFID选通方向由RFID_R200_AB_SWAP_ENABLE单独控制。
 * MemoryMsgA/B、界面区域、报警归属和业务状态始终保持逻辑A/B不变。
 */
#ifndef HANDLE_PHYSICAL_AB_SWAP_ENABLE
#define HANDLE_PHYSICAL_AB_SWAP_ENABLE    0U
#endif

#if ((HANDLE_PHYSICAL_AB_SWAP_ENABLE != 0U) && (HANDLE_PHYSICAL_AB_SWAP_ENABLE != 1U))
#error "HANDLE_PHYSICAL_AB_SWAP_ENABLE must be 0U or 1U"
#endif

/* 手柄逻辑通道编号与现有CHANNEL_A/CHANNEL_B保持一致，集中定义避免各模块重复写数字。 */
#define BOARD_PROFILE_HANDLE_CHANNEL_A    1U
#define BOARD_PROFILE_HANDLE_CHANNEL_B    2U

/**
 * 函数功能：把手柄逻辑A/B通道换算成当前接线对应的物理A/B通道。
 * 输入参数：logical_channel，手柄业务使用的逻辑通道编号，1表示A，2表示B。
 * 返回参数：物理通道编号；非A/B编号原样返回，由调用方沿用原有异常处理。
 */
static inline uint8_t BoardProfile_MapHandlePhysicalChannel(uint8_t logical_channel)
{
#if (HANDLE_PHYSICAL_AB_SWAP_ENABLE == 1U)
    /* 当前线束镜像安装，逻辑A必须访问原物理B接口。 */
    if (logical_channel == BOARD_PROFILE_HANDLE_CHANNEL_A)
    {
        return BOARD_PROFILE_HANDLE_CHANNEL_B;
    }

    /* 当前线束镜像安装，逻辑B必须访问原物理A接口。 */
    if (logical_channel == BOARD_PROFILE_HANDLE_CHANNEL_B)
    {
        return BOARD_PROFILE_HANDLE_CHANNEL_A;
    }
#endif

    /* 关闭交换或输入不是A/B时保持原编号，避免改变调用方既有异常语义。 */
    return logical_channel;
}

#endif /* __BOARD_PROFILE_H */
