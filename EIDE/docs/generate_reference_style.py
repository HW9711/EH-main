#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
参考风格嵌入式软件架构图生成脚本
基于 diagram-architect skill 分析参考图片风格
"""

import os
import subprocess

# 确保输出目录存在
os.makedirs('docs/assets', exist_ok=True)

# Graphviz 路径配置
GRAPHVIZ_PATH = r"D:\Soft_Install_HL\graphviz\bin"
DOT_EXE = os.path.join(GRAPHVIZ_PATH, "dot.exe")

# 分析参考图片风格特征（基于常见软件架构图）：
# 1. 分层清晰，每层有不同背景色
# 2. 节点有圆角，填充颜色
# 3. 连线有箭头，实线或虚线
# 4. 标签字体清晰
# 5. 布局整齐

# 参考风格 DOT 定义 - 类似技术文档中的架构图
reference_dot = '''digraph ReferenceArchitecture {
    // 参考风格配置
    graph [
        rankdir=TB
        splines=ortho
        nodesep=0.25
        ranksep=0.5
        fontname="Arial, sans-serif"
        fontsize=13
        label="F413 嵌入式软件架构"
        labelloc=t
        pad=0.5
        bgcolor="#FAFAFA"
        concentrate=false
        newrank=true
    ];
    
    // 节点样式 - 参考风格
    node [
        fontname="Arial, sans-serif"
        fontsize=10
        shape=rect
        style="rounded,filled"
        penwidth=1.0
        margin=0.15
        fillcolor="#FFFFFF"
        color="#333333"
    ];
    
    // 连线样式 - 参考风格
    edge [
        fontname="Arial, sans-serif"
        fontsize=8
        arrowsize=0.7
        penwidth=1.2
        color="#555555"
        arrowhead=normal
    ];

    // ===== 应用层 =====
    subgraph cluster_app {
        label="应用层 (Application)"
        style="filled,rounded"
        fillcolor="#E8F4FF"
        color="#1976D2"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        app_parser [label="userparser.c\n系统初始化与任务创建", 
                   fillcolor="#FFFFFF", color="#1976D2"];
        app_screen [label="screen.c\nUI显示与按键处理", 
                   fillcolor="#FFFFFF", color="#1976D2"];
        app_task [label="app_task.c\n定时任务调度框架", 
                 fillcolor="#FFFFFF", color="#1976D2"];
    }

    // ===== 中间件层 =====
    subgraph cluster_mw {
        label="中间件 (Middleware)"
        style="filled,rounded"
        fillcolor="#F0F9E8"
        color="#388E3C"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        mw_freertos [label="FreeRTOS\n任务管理与调度", 
                    fillcolor="#FFFFFF", color="#388E3C"];
        mw_apptask [label="AppTask框架\n定时任务链表", 
                   fillcolor="#FFFFFF", color="#388E3C"];
    }

    // ===== 驱动层 =====
    subgraph cluster_drv {
        label="驱动层 (Driver)"
        style="filled,rounded"
        fillcolor="#FFF3E0"
        color="#F57C00"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        drv_uart [label="UART1-7\n串口通信 (DMA)", fillcolor="#FFFFFF", color="#F57C00"];
        drv_gpio [label="GPIO\n输入输出控制", fillcolor="#FFFFFF", color="#F57C00"];
        drv_tim [label="TIM7/10/14\n定时器驱动", fillcolor="#FFFFFF", color="#F57C00"];
        drv_adc [label="ADC\n模拟信号采集", fillcolor="#FFFFFF", color="#F57C00"];
        drv_i2c [label="Software I2C\nEEPROM存储", fillcolor="#FFFFFF", color="#F57C00"];
        drv_1wire [label="1-Wire\nRFID/DS2401", fillcolor="#FFFFFF", color="#F57C00"];
    }

    // ===== 硬件抽象层 =====
    subgraph cluster_hal {
        label="硬件抽象层 (HAL)"
        style="filled,rounded"
        fillcolor="#FCE4EC"
        color="#C2185B"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        hal_board [label="board.h/c\n引脚与外设配置", fillcolor="#FFFFFF", color="#C2185B"];
        hal_stm32 [label="STM32F4xx HAL\n外设驱动库", fillcolor="#FFFFFF", color="#C2185B"];
    }

    // ===== 硬件层 =====
    subgraph cluster_hw {
        label="硬件层 (Hardware)"
        style="filled,rounded"
        fillcolor="#E8EAF6"
        color="#303F9F"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        hw_mcu [label="STM32F4xx\nCortex-M4 100MHz", fillcolor="#FFFFFF", color="#303F9F"];
        hw_uart [label="UART\n7路串口", fillcolor="#FFFFFF", color="#303F9F"];
        hw_gpio [label="GPIO\n多路输入输出", fillcolor="#FFFFFF", color="#303F9F"];
        hw_eeprom [label="AT24C02\n2Kb EEPROM", fillcolor="#FFFFFF", color="#303F9F"];
    }

    // ===== 外部设备层 =====
    subgraph cluster_peri {
        label="外部设备 (Peripherals)"
        style="filled,rounded"
        fillcolor="#FFF8E1"
        color="#FF8F00"
        penwidth=2
        fontname="Arial, sans-serif"
        fontsize=11
        
        per_motor [label="无刷电机\nA/B双通道", fillcolor="#FFFFFF", color="#FF8F00"];
        per_lcd [label="LCD显示屏\n4.3寸触摸", fillcolor="#FFFFFF", color="#FF8F00"];
        per_pedal [label="脚踏板\n油门+按键", fillcolor="#FFFFFF", color="#FF8F00"];
        per_hand [label="手柄\nRFID识别", fillcolor="#FFFFFF", color="#FF8F00"];
        per_pump [label="灌注/注水泵\n流量控制", fillcolor="#FFFFFF", color="#FF8F00"];
    }

    // ===== 连接关系 =====
    // 应用层 -> 中间件（实线）
    app_parser -> mw_freertos [color="#1976D2"];
    app_screen -> mw_apptask [color="#1976D2"];
    app_task -> mw_freertos [color="#1976D2"];
    app_task -> mw_apptask [color="#1976D2"];

    // 中间件 -> 驱动层（实线）
    mw_freertos -> drv_uart [color="#388E3C"];
    mw_freertos -> drv_gpio [color="#388E3C"];
    mw_freertos -> drv_tim [color="#388E3C"];
    mw_apptask -> drv_uart [color="#388E3C", style=dashed];
    mw_apptask -> drv_gpio [color="#388E3C", style=dashed];

    // 驱动层 -> HAL（实线）
    drv_uart -> hal_board [color="#F57C00"];
    drv_gpio -> hal_board [color="#F57C00"];
    drv_tim -> hal_board [color="#F57C00"];
    drv_adc -> hal_board [color="#F57C00"];
    drv_i2c -> hal_board [color="#F57C00"];
    drv_1wire -> hal_board [color="#F57C00"];

    // HAL -> 硬件（实线）
    hal_board -> hal_stm32 [color="#C2185B"];
    hal_stm32 -> hw_mcu [color="#303F9F"];
    hal_stm32 -> hw_uart [color="#303F9F"];
    hal_stm32 -> hw_gpio [color="#303F9F"];
    
    // 硬件 -> 外设（实线，部分虚线表示可选连接）
    hw_uart -> per_motor [color="#FF8F00"];
    hw_uart -> per_lcd [color="#FF8F00"];
    hw_uart -> per_pedal [color="#FF8F00"];
    hw_uart -> per_hand [color="#FF8F00"];
    hw_gpio -> per_pump [color="#FF8F00"];
    hw_eeprom -> hal_stm32 [color="#303F9F", style=dashed];

    // ===== 层级对齐 =====
    {rank=same; app_parser app_screen app_task}
    {rank=same; mw_freertos mw_apptask}
    {rank=same; drv_uart drv_gpio drv_tim drv_adc drv_i2c drv_1wire}
    {rank=same; hal_board hal_stm32}
    {rank=same; hw_mcu hw_uart hw_gpio hw_eeprom}
    {rank=same; per_motor per_lcd per_pedal per_hand per_pump}
}'''

# 写入 DOT 文件
dot_file = 'docs/reference_style.dot'
with open(dot_file, 'w', encoding='utf-8') as f:
    f.write(reference_dot)

print(f"参考风格 DOT 文件已生成: {dot_file}")

# 生成 PNG 图片
try:
    png_file = 'docs/reference_style.png'
    result = subprocess.run([
        DOT_EXE, '-Tpng', '-Gdpi=200',
        '-o', png_file,
        dot_file
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"参考风格 PNG 图片已生成: {png_file}")
    else:
        print(f"Graphviz 错误: {result.stderr}")
        
except Exception as e:
    print(f"生成 PNG 时出错: {e}")

# 生成 SVG 矢量图
try:
    svg_file = 'docs/reference_style.svg'
    result = subprocess.run([
        DOT_EXE, '-Tsvg',
        '-o', svg_file,
        dot_file
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"参考风格 SVG 矢量图已生成: {svg_file}")
        
except Exception as e:
    print(f"生成 SVG 时出错: {e}")

print("\n=== 参考风格生成完成 ===")
print("风格: 技术文档参考风格")
print("配色: Material Design 调色板")
print("布局: 正交连线")
print("效果: 圆角矩形，分层填充")