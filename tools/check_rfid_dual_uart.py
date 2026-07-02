# -*- coding: utf-8 -*-
"""检查 RFID 双串口改造是否覆盖关键工程配置。"""

from pathlib import Path
import re
import sys


# 工程根目录由脚本所在位置反推，避免在 EIDE 或根目录执行时路径不同。
ROOT = Path(__file__).resolve().parents[1]


def read_text(rel_path: str) -> str:
    """读取文本文件；源码里存在历史编码内容，因此只做关键 ASCII/UTF-8 片段校验。"""
    return (ROOT / rel_path).read_text(encoding="utf-8", errors="ignore")


def require(condition: bool, message: str, failures: list[str]) -> None:
    """记录失败项，便于一次运行看到所有缺口。"""
    if not condition:
        failures.append(message)


def require_file(rel_path: str, failures: list[str]) -> str:
    """确认文件存在并返回内容，缺失时记录失败但不中断后续检查。"""
    path = ROOT / rel_path
    require(path.exists(), f"缺少文件: {rel_path}", failures)
    return read_text(rel_path) if path.exists() else ""


def main() -> int:
    """主检查入口：逐项确认 UART9、宏切换、RFID 路由和工程清单。"""
    failures: list[str] = []

    board_profile = require_file("User/Hardware/include/board_profile.h", failures)
    board_h = require_file("User/board/board.h", failures)
    usart_h = require_file("Inc/usart.h", failures)
    usart_c = require_file("Src/usart.c", failures)
    dma_c = require_file("Src/dma.c", failures)
    it_h = require_file("Inc/stm32f4xx_it.h", failures)
    it_c = require_file("Src/stm32f4xx_it.c", failures)
    bsp_uart_h = require_file("User/Hardware/include/bsp_uart.h", failures)
    bsp_uart_c = require_file("User/Hardware/Bsp/bsp_uart.c", failures)
    userparser_c = require_file("User/Application/Src/userparser.c", failures)
    ssc_rfid_c = require_file("User/Application/Beep/sscRFID.c", failures)
    uart9_h = require_file("User/Peripheral/include/uart9.h", failures)
    uart9_c = require_file("User/Peripheral/uart/uart9.c", failures)
    eide_yml = require_file("EIDE/.eide/eide.yml", failures)
    eide_params = require_file("EIDE/build/MainCtrlF413MXOs/builder.params", failures)
    root_params = require_file("build/MainCtrlF413MXOs/builder.params", failures)
    uvprojx = require_file("MDK-ARM/MainCtrlF413MXOs.uvprojx", failures)
    uvoptx = require_file("MDK-ARM/MainCtrlF413MXOs.uvoptx", failures)

    require("#define RFID_USE_DUAL_UART_MODE" in board_profile,
            "缺少 RFID_USE_DUAL_UART_MODE 模式宏", failures)
    require("BOARD_UART9_TX_PORT" in board_h and "GPIOD" in board_h,
            "board.h 缺少 UART9 TX 端口配置", failures)
    require("BOARD_UART9_TX_PIN" in board_h and "GPIO_PIN_14" in board_h,
            "board.h 缺少 UART9 TX=PD14 配置", failures)
    require("BOARD_UART9_RX_PIN" in board_h and "GPIO_PIN_15" in board_h,
            "board.h 缺少 UART9 RX=PD15 配置", failures)
    require("GPIO_AF11_UART9" in board_h,
            "board.h 缺少 UART9 AF11 复用配置", failures)
    require("BOARD_UART9_DMA_RX" in board_h and "DMA2_Stream7" in board_h,
            "board.h 缺少 UART9_RX DMA2_Stream7 配置", failures)
    require("BOARD_UART9_DMA_CHANNEL" in board_h and "DMA_CHANNEL_0" in board_h,
            "board.h 缺少 UART9_RX DMA Channel 0 配置", failures)

    require("extern UART_HandleTypeDef huart9" in usart_h,
            "usart.h 缺少 huart9 声明", failures)
    require("void MX_UART9_Init(void);" in usart_h,
            "usart.h 缺少 MX_UART9_Init 声明", failures)
    require("UART_HandleTypeDef huart9" in usart_c,
            "usart.c 缺少 huart9 句柄", failures)
    require("DMA_HandleTypeDef hdma_uart9_rx" in usart_c,
            "usart.c 缺少 hdma_uart9_rx 句柄", failures)
    require("huart9.Instance = UART9" in usart_c,
            "usart.c 缺少 UART9 初始化函数", failures)
    require("BOARD_UART9_DMA_RX" in usart_c and "BOARD_UART9_DMA_CHANNEL" in usart_c,
            "usart.c 缺少 UART9 DMA 初始化", failures)
    require("HAL_NVIC_SetPriority(UART9_IRQn" in usart_c,
            "usart.c 缺少 UART9 中断使能", failures)

    require("DMA2_Stream7_IRQn" in dma_c,
            "dma.c 缺少 DMA2_Stream7 中断配置", failures)
    require("void DMA2_Stream7_IRQHandler(void)" in it_h,
            "stm32f4xx_it.h 缺少 DMA2_Stream7_IRQHandler 声明", failures)
    require("void UART9_IRQHandler(void)" in it_h,
            "stm32f4xx_it.h 缺少 UART9_IRQHandler 声明", failures)
    require("HAL_DMA_IRQHandler(&hdma_uart9_rx)" in it_c,
            "stm32f4xx_it.c 缺少 UART9 RX DMA 中断处理", failures)
    require("HAL_UART_IRQHandler(&huart9)" in it_c,
            "stm32f4xx_it.c 缺少 UART9 中断处理", failures)

    require("BSP_UART_PORT_9" in bsp_uart_h and "BSP_UART_PORT_9" in bsp_uart_c,
            "BSP UART 层缺少端口 9", failures)
    require("Uart9_Init" in uart9_h and "Uart9_Init" in uart9_c,
            "缺少 UART9 应用层初始化接口", failures)
    require("Uart9_SendPacket" in uart9_h and "BSP_UART_PORT_9" in uart9_c,
            "缺少 UART9 应用层发送接口或 BSP 绑定", failures)
    require("Uart9_DMARecvDataPeek" in uart9_h and "Uart9_DMARecvDataPeek" in uart9_c,
            "缺少 UART9 DMA 接收读取接口", failures)

    require("uart9.h" in userparser_c and "Uart9_Init" in userparser_c,
            "Userparser_Init 缺少 UART9 初始化调用", failures)
    require("MX_UART9_Init" in require_file("Src/main.c", failures),
            "main.c 缺少 MX_UART9_Init 调用", failures)
    require("uart9.h" in ssc_rfid_c and "RFID_USE_DUAL_UART_MODE" in ssc_rfid_c,
            "sscRFID.c 缺少双串口模式宏或 UART9 头文件", failures)
    require("Uart9_SendPacket" in ssc_rfid_c and "Uart9_DMARecvDataPeek" in ssc_rfid_c,
            "sscRFID.c 未把 B 通道路由到 UART9", failures)
    require("R200_K8_SELECT_A" in ssc_rfid_c and "#if (RFID_USE_DUAL_UART_MODE == 0U)" in ssc_rfid_c,
            "sscRFID.c 未保留旧 R200-K8 切换模式", failures)

    for rel_path, content in {
        "EIDE/.eide/eide.yml": eide_yml,
        "EIDE/build/MainCtrlF413MXOs/builder.params": eide_params,
        "build/MainCtrlF413MXOs/builder.params": root_params,
        "MDK-ARM/MainCtrlF413MXOs.uvprojx": uvprojx,
        "MDK-ARM/MainCtrlF413MXOs.uvoptx": uvoptx,
    }.items():
        require("uart9.c" in content, f"{rel_path} 缺少 uart9.c 工程清单项", failures)

    if failures:
        print("RFID 双串口静态检查失败：")
        for item in failures:
            print(f"- {item}")
        return 1

    print("RFID 双串口静态检查通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
