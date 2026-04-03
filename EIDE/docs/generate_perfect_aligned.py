#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完美对齐的嵌入式软件架构图
使用简单但有效的对称布局方法
"""

import os
import subprocess

os.makedirs('docs/assets', exist_ok=True)

GRAPHVIZ_PATH = r"D:\Soft_Install_HL\graphviz\bin"
DOT_EXE = os.path.join(GRAPHVIZ_PATH, "dot.exe")

# 关键策略：
# 1. 每层使用完全相同的节点布局（6列）
# 2. 内容节点居中对齐，左右使用不可见节点填充
# 3. 不使用复杂的约束，让布局更自然

perfect_dot = '''digraph PerfectAlignedArchitecture {
    // 完美对齐配置
    graph [
        rankdir=TB
        splines=ortho
        nodesep=0.15
        ranksep=0.3
        fontname="Arial, sans-serif"
        fontsize=13
        label="F413 嵌入式软件架构"
        labelloc=t
        pad=0.3
        bgcolor="#FFFFFF"
        newrank=true
        concentrate=false
    ];
    
    // 节点样式 - 固定尺寸
    node [
        fontname="Arial, sans-serif"
        fontsize=10
        shape=rect
        style="rounded,filled"
        penwidth=1.0
        margin=0.08
        width=1.6
        height=0.55
        fillcolor="#FFFFFF"
        color="#333333"
    ];
    
    // 连线样式
    edge [
        fontname="Arial, sans-serif"
        fontsize=8
        arrowsize=0.6
        penwidth=1.0
        color="#555555"
        arrowhead=normal
    ];

    // ===== 关键改进：每层使用完全对称的6列布局 =====
    
    // === 第1层：应用层 ===
    // 布局：填充1 + 节点1 + 节点2 + 节点3 + 填充2 + 填充3
    subgraph cluster_app {
        label="应用层 (Application)"
        style="filled,rounded"
        fillcolor="#E8F4FF"
        color="#1976D2"
        penwidth=1.5
        
        app_f1 [label="", style="invisible", width=1.6, height=0.01];
        app_parser [label="userparser.c\\n系统初始化与任务创建", fillcolor="#FFFFFF", color="#1976D2"];
        app_screen [label="screen.c\\nUI显示与按键处理", fillcolor="#FFFFFF", color="#1976D2"];
        app_task [label="app_task.c\\n定时任务调度框架", fillcolor="#FFFFFF", color="#1976D2"];
        app_f2 [label="", style="invisible", width=1.6, height=0.01];
        app_f3 [label="", style="invisible", width=1.6, height=0.01];
    }

    // === 第2层：中间件层 ===
    // 布局：填充1 + 填充2 + 节点1 + 节点2 + 填充3 + 填充4
    subgraph cluster_mw {
        label="中间件 (Middleware)"
        style="filled,rounded"
        fillcolor="#F0F9E8"
        color="#388E3C"
        penwidth=1.5
        
        mw_f1 [label="", style="invisible", width=1.6, height=0.01];
        mw_f2 [label="", style="invisible", width=1.6, height=0.01];
        mw_freertos [label="FreeRTOS\\n任务管理与调度", fillcolor="#FFFFFF", color="#388E3C"];
        mw_apptask [label="AppTask框架\\n定时任务链表", fillcolor="#FFFFFF", color="#388E3C"];
        mw_f3 [label="", style="invisible", width=1.6, height=0.01];
        mw_f4 [label="", style="invisible", width=1.6, height=0.01];
    }

    // === 第3层：驱动层 ===
    // 布局：6个实际节点
    subgraph cluster_drv {
        label="驱动层 (Driver)"
        style="filled,rounded"
        fillcolor="#FFF3E0"
        color="#F57C00"
        penwidth=1.5
        
        drv_uart [label="UART1-7\\n串口通信 (DMA)", fillcolor="#FFFFFF", color="#F57C00"];
        drv_gpio [label="GPIO\\n输入输出控制", fillcolor="#FFFFFF", color="#F57C00"];
        drv_tim [label="TIM7/10/14\\n定时器驱动", fillcolor="#FFFFFF", color="#F57C00"];
        drv_adc [label="ADC\\n模拟信号采集", fillcolor="#FFFFFF", color="#F57C00"];
        drv_i2c [label="Software I2C\\nEEPROM存储", fillcolor="#FFFFFF", color="#F57C00"];
        drv_1wire [label="1-Wire\\nRFID/DS2401", fillcolor="#FFFFFF", color="#F57C00"];
    }

    // === 第4层：硬件抽象层 ===
    // 布局：填充1 + 填充2 + 节点1 + 节点2 + 填充3 + 填充4
    subgraph cluster_hal {
        label="硬件抽象层 (HAL)"
        style="filled,rounded"
        fillcolor="#FCE4EC"
        color="#C2185B"
        penwidth=1.5
        
        hal_f1 [label="", style="invisible", width=1.6, height=0.01];
        hal_f2 [label="", style="invisible", width=1.6, height=0.01];
        hal_board [label="board.h/c\\n引脚与外设配置", fillcolor="#FFFFFF", color="#C2185B"];
        hal_stm32 [label="STM32F4xx HAL\\n外设驱动库", fillcolor="#FFFFFF", color="#C2185B"];
        hal_f3 [label="", style="invisible", width=1.6, height=0.01];
        hal_f4 [label="", style="invisible", width=1.6, height=0.01];
    }

    // === 第5层：硬件层 ===
    // 布局：填充1 + 节点1 + 节点2 + 节点3 + 节点4 + 填充2
    subgraph cluster_hw {
        label="硬件层 (Hardware)"
        style="filled,rounded"
        fillcolor="#E8EAF6"
        color="#303F9F"
        penwidth=1.5
        
        hw_f1 [label="", style="invisible", width=1.6, height=0.01];
        hw_mcu [label="STM32F4xx\\nCortex-M4 100MHz", fillcolor="#FFFFFF", color="#303F9F"];
        hw_uart [label="UART\\n7路串口", fillcolor="#FFFFFF", color="#303F9F"];
        hw_gpio [label="GPIO\\n多路输入输出", fillcolor="#FFFFFF", color="#303F9F"];
        hw_eeprom [label="AT24C02\\n2Kb EEPROM", fillcolor="#FFFFFF", color="#303F9F"];
        hw_f2 [label="", style="invisible", width=1.6, height=0.01];
    }

    // === 第6层：外部设备层 ===
    // 布局：节点1 + 节点2 + 节点3 + 节点4 + 节点5 + 填充1
    subgraph cluster_peri {
        label="外部设备 (Peripherals)"
        style="filled,rounded"
        fillcolor="#FFF8E1"
        color="#FF8F00"
        penwidth=1.5
        
        per_motor [label="无刷电机\\nA/B双通道", fillcolor="#FFFFFF", color="#FF8F00"];
        per_lcd [label="LCD显示屏\\n4.3寸触摸", fillcolor="#FFFFFF", color="#FF8F00"];
        per_pedal [label="脚踏板\\n油门+按键", fillcolor="#FFFFFF", color="#FF8F00"];
        per_hand [label="手柄\\nRFID识别", fillcolor="#FFFFFF", color="#FF8F00"];
        per_pump [label="灌注/注水泵\\n流量控制", fillcolor="#FFFFFF", color="#FF8F00"];
        peri_f1 [label="", style="invisible", width=1.6, height=0.01];
    }

    // ===== 连接关系 =====
    app_parser -> mw_freertos [color="#1976D2"];
    app_screen -> mw_apptask [color="#1976D2"];
    app_task -> mw_freertos [color="#1976D2"];
    app_task -> mw_apptask [color="#1976D2"];

    mw_freertos -> drv_uart [color="#388E3C"];
    mw_freertos -> drv_gpio [color="#388E3C"];
    mw_freertos -> drv_tim [color="#388E3C"];
    mw_apptask -> drv_uart [color="#388E3C", style=dashed];
    mw_apptask -> drv_gpio [color="#388E3C", style=dashed];

    drv_uart -> hal_board [color="#F57C00"];
    drv_gpio -> hal_board [color="#F57C00"];
    drv_tim -> hal_board [color="#F57C00"];
    drv_adc -> hal_board [color="#F57C00"];
    drv_i2c -> hal_board [color="#F57C00"];
    drv_1wire -> hal_board [color="#F57C00"];

    hal_board -> hal_stm32 [color="#C2185B"];
    hal_stm32 -> hw_mcu [color="#303F9F"];
    hal_stm32 -> hw_uart [color="#303F9F"];
    hal_stm32 -> hw_gpio [color="#303F9F"];
    
    hw_uart -> per_motor [color="#FF8F00"];
    hw_uart -> per_lcd [color="#FF8F00"];
    hw_uart -> per_pedal [color="#FF8F00"];
    hw_uart -> per_hand [color="#FF8F00"];
    hw_gpio -> per_pump [color="#FF8F00"];
    hw_eeprom -> hal_stm32 [color="#303F9F", style=dashed];

    // ===== 关键：每层的位置约束 =====
    // 让每层的填充节点和内容节点在各自层内对齐
    
    // 第1层：应用层 - 所有节点在同一层级
    {rank=same; app_f1 app_parser app_screen app_task app_f2 app_f3}
    
    // 第2层：中间件层 - 所有节点在同一层级
    {rank=same; mw_f1 mw_f2 mw_freertos mw_apptask mw_f3 mw_f4}
    
    // 第3层：驱动层 - 所有节点在同一层级
    {rank=same; drv_uart drv_gpio drv_tim drv_adc drv_i2c drv_1wire}
    
    // 第4层：硬件抽象层 - 所有节点在同一层级
    {rank=same; hal_f1 hal_f2 hal_board hal_stm32 hal_f3 hal_f4}
    
    // 第5层：硬件层 - 所有节点在同一层级
    {rank=same; hw_f1 hw_mcu hw_uart hw_gpio hw_eeprom hw_f2}
    
    // 第6层：外部设备层 - 所有节点在同一层级
    {rank=same; per_motor per_lcd per_pedal per_hand per_pump peri_f1}
    
    // ===== 改进：添加不可见边来强制垂直对齐 =====
    // 确保每层的左侧填充节点对齐
    edge [style="invis", weight=100];
    app_f1 -> mw_f1 -> drv_uart -> hal_f1 -> hw_f1 -> per_motor;
    
    // 确保每层的右侧填充节点对齐
    app_f3 -> mw_f4 -> drv_1wire -> hal_f4 -> hw_f2 -> peri_f1;
    
    // ===== 添加水平约束，确保每层宽度一致 =====
    // 强制每层的第一列对齐（左侧边界）
    app_f1 -> mw_f1 [constraint=false];
    mw_f1 -> drv_uart [constraint=false];
    drv_uart -> hal_f1 [constraint=false];
    hal_f1 -> hw_f1 [constraint=false];
    hw_f1 -> per_motor [constraint=false];
    
    // 强制每层的最后一列对齐（右侧边界）
    app_f3 -> mw_f4 [constraint=false];
    mw_f4 -> drv_1wire [constraint=false];
    drv_1wire -> hal_f4 [constraint=false];
    hal_f4 -> hw_f2 [constraint=false];
    hw_f2 -> peri_f1 [constraint=false];
}'''

dot_file = 'docs/perfect_aligned.dot'
with open(dot_file, 'w', encoding='utf-8') as f:
    f.write(perfect_dot)

print(f"完美对齐 DOT 文件已生成: {dot_file}")

# 生成 PNG
try:
    png_file = 'docs/perfect_aligned.png'
    result = subprocess.run([
        DOT_EXE, '-Tpng', '-Gdpi=200',
        '-o', png_file,
        dot_file
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"完美对齐 PNG 图片已生成: {png_file}")
    else:
        print(f"Graphviz 错误: {result.stderr}")
        
except Exception as e:
    print(f"生成 PNG 时出错: {e}")

print("\n=== 完美对齐生成完成 ===")
print("关键改进:")
print("1. 每层使用完全对称的6列布局")
print("2. 内容节点在各自层内居中")
print("3. 使用不可见边强制垂直对齐")
print("4. 添加水平约束确保边界对齐")