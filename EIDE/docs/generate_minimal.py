#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
极简风格嵌入式软件架构图生成脚本
使用 diagram-architect skill 的极简风格
"""

import os
import subprocess

# 确保输出目录存在
os.makedirs('docs/assets', exist_ok=True)

# Graphviz 路径配置 - 使用系统安装的 Graphviz
GRAPHVIZ_PATH = r"D:\Soft_Install_HL\graphviz\bin"
DOT_EXE = os.path.join(GRAPHVIZ_PATH, "dot.exe")

# 极简风格 DOT 定义 - 更简约的版本
minimal_dot = '''digraph MinimalArchitecture {
    // 极简配置
    graph [
        rankdir=TB
        nodesep=0.3
        ranksep=0.4
        fontname="Arial"
        fontsize=12
        label="F413 嵌入式软件架构"
        labelloc=t
        pad=0.3
        bgcolor="#FFFFFF"
        concentrate=true
    ];
    
    // 节点样式 - 极简
    node [
        fontname="Arial"
        fontsize=9
        shape=box
        style="solid"
        penwidth=1.0
        margin=0.08
        color="#333333"
    ];
    
    // 连线样式 - 极简
    edge [
        fontname="Arial"
        fontsize=8
        arrowsize=0.5
        penwidth=0.8
        color="#666666"
    ];

    // ===== 应用层 =====
    subgraph cluster_app {
        label="应用层"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        app_parser [label="系统初始化"];
        app_screen [label="UI显示"];
        app_task [label="任务调度"];
    }

    // ===== 中间件层 =====
    subgraph cluster_mw {
        label="中间件"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        mw_freertos [label="FreeRTOS"];
        mw_apptask [label="AppTask"];
    }

    // ===== 驱动层 =====
    subgraph cluster_drv {
        label="驱动层"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        drv_uart [label="UART通信"];
        drv_gpio [label="GPIO控制"];
        drv_tim [label="定时器"];
        drv_adc [label="ADC采集"];
        drv_i2c [label="EEPROM"];
        drv_1wire [label="RFID"];
    }

    // ===== 硬件抽象层 =====
    subgraph cluster_hal {
        label="硬件抽象层"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        hal_board [label="引脚配置"];
        hal_stm32 [label="STM32 HAL"];
    }

    // ===== 硬件层 =====
    subgraph cluster_hw {
        label="硬件层"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        hw_mcu [label="STM32F4xx"];
        hw_uart [label="UART"];
        hw_gpio [label="GPIO"];
        hw_eeprom [label="EEPROM"];
    }

    // ===== 外部设备层 =====
    subgraph cluster_peri {
        label="外部设备"
        style="solid"
        color="#333333"
        penwidth=1.2
        
        per_motor [label="无刷电机"];
        per_lcd [label="LCD显示屏"];
        per_pedal [label="脚踏板"];
        per_hand [label="手柄"];
        per_pump [label="水泵"];
    }

    // ===== 连接关系 =====
    app_parser -> mw_freertos;
    app_screen -> mw_apptask;
    app_task -> mw_freertos;
    app_task -> mw_apptask;

    mw_freertos -> drv_uart;
    mw_freertos -> drv_gpio;
    mw_freertos -> drv_tim;
    mw_apptask -> drv_uart;
    mw_apptask -> drv_gpio;

    drv_uart -> hal_board;
    drv_gpio -> hal_board;
    drv_tim -> hal_board;
    drv_adc -> hal_board;
    drv_i2c -> hal_board;
    drv_1wire -> hal_board;

    hal_board -> hal_stm32;
    hal_stm32 -> hw_mcu;
    hal_stm32 -> hw_uart;
    hal_stm32 -> hw_gpio;
    
    hw_uart -> per_motor;
    hw_uart -> per_lcd;
    hw_uart -> per_pedal;
    hw_uart -> per_hand;
    hw_gpio -> per_pump;
    hw_eeprom -> hal_stm32;

    // ===== 层级对齐 =====
    {rank=same; app_parser app_screen app_task}
    {rank=same; mw_freertos mw_apptask}
    {rank=same; drv_uart drv_gpio drv_tim drv_adc drv_i2c drv_1wire}
    {rank=same; hal_board hal_stm32}
    {rank=same; hw_mcu hw_uart hw_gpio hw_eeprom}
    {rank=same; per_motor per_lcd per_pedal per_hand per_pump}
}'''

# 写入 DOT 文件
dot_file = 'docs/minimal_simple.dot'
with open(dot_file, 'w', encoding='utf-8') as f:
    f.write(minimal_dot)

print(f"极简 DOT 文件已生成: {dot_file}")

# 生成 PNG 图片
try:
    png_file = 'docs/minimal_simple.png'
    result = subprocess.run([
        DOT_EXE, '-Tpng', '-Gdpi=150',
        '-o', png_file,
        dot_file
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"极简 PNG 图片已生成: {png_file}")
    else:
        print(f"Graphviz 错误: {result.stderr}")
        
except Exception as e:
    print(f"生成 PNG 时出错: {e}")

# 生成 SVG 矢量图
try:
    svg_file = 'docs/minimal_simple.svg'
    result = subprocess.run([
        DOT_EXE, '-Tsvg',
        '-o', svg_file,
        dot_file
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"极简 SVG 矢量图已生成: {svg_file}")
        
except Exception as e:
    print(f"生成 SVG 时出错: {e}")

print("\n=== 极简风格生成完成 ===")
print("风格: 极简主义")
print("颜色: 黑白灰")
print("布局: 分层架构")
print("连线: 简约箭头")