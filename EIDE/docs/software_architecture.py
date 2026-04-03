#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
软件架构图生成脚本
使用 Graphviz 生成矢量架构图
"""

import os

os.makedirs('docs/assets', exist_ok=True)

# Okabe-Ito 颜色盲友好调色板
OKABE_ITO = {
    'orange': '#E69F00',
    'sky_blue': '#56B4E9',
    'bluish_green': '#009E73',
    'blue': '#0072B2',
    'vermillion': '#D55E00',
    'reddish_purple': '#CC79A7',
    'black': '#000000'
}

# 所有层统一宽度配置
LAYOUT_CONFIG = {
    'nodesep': 0.2,
    'ranksep': 0.5,
    'node_width': 2.2,
    'node_height': 0.7,
    'filler_width': 2.2
}

# 不可见填充节点生成
def make_filler(name):
    return f'{name} [label="", style="invisible", width={LAYOUT_CONFIG["filler_width"]}, height=0.01];'

# 统一节点宽度
def node_attr(width=LAYOUT_CONFIG['node_width'], height=LAYOUT_CONFIG['node_height']):
    return f'width={width}, height={height}'

# Graphviz DOT - 所有层宽度一致，居中对齐
dot_content = f'''digraph Architecture {{
    graph [
        rankdir=TB
        nodesep={LAYOUT_CONFIG['nodesep']}
        ranksep={LAYOUT_CONFIG['ranksep']}
        fontname="Microsoft YaHei"
        fontsize=14
        label="F413 嵌入式软件架构图"
        labelloc=t
        pad=0.5
        bgcolor="#FFFFFF"
        newrank=true
    ];
    
    node [
        fontname="Microsoft YaHei"
        fontsize=10
        shape=rect
        style="rounded,filled"
        penwidth=1.2
        margin=0.1
        {node_attr()}
    ];
    
    edge [
        fontname="Microsoft YaHei"
        fontsize=8
        arrowsize=0.7
        penwidth=1.2
        color="#666666"
        style=dashed
    ];

    // ===== 统一居中对齐配置 =====
    // 使用rank=same强制所有层在同一水平位置
    
    // ===== 应用层 (3个节点) =====
    subgraph cluster_app {{
        label="应用层 (Application)"
        style="dashed,filled"
        fillcolor="#E3F2FD"
        color="{OKABE_ITO['blue']}"
        penwidth=2
        
        // 居中：左边1个填充 + 3节点 + 右边1个填充
        app_fill1 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        app_parser [label="userparser.c\\n系统初始化与任务创建", fillcolor="#BBDEFB", color="{OKABE_ITO['blue']}"];
        app_screen [label="screen.c\\nUI显示与按键处理", fillcolor="#BBDEFB", color="{OKABE_ITO['blue']}"];
        app_task [label="app_task.c\\n定时任务调度框架", fillcolor="#BBDEFB", color="{OKABE_ITO['blue']}"];
        app_fill2 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
    }}

    // ===== 中间件层 (2个节点) =====
    subgraph cluster_mw {{
        label="中间件 (Middleware)"
        style="dashed,filled"
        fillcolor="#E8F5E9"
        color="{OKABE_ITO['bluish_green']}"
        penwidth=2
        
        // 居中：左边2个填充 + 2节点 + 右边2个填充
        mw_fill1 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        mw_fill2 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        mw_freertos [label="FreeRTOS\\n任务管理与调度", fillcolor="#C8E6C9", color="{OKABE_ITO['bluish_green']}"];
        mw_apptask [label="AppTask框架\\n定时任务链表", fillcolor="#C8E6C9", color="{OKABE_ITO['bluish_green']}"];
        mw_fill3 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        mw_fill4 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
    }}

    // ===== 驱动层 (6个节点) =====
    subgraph cluster_drv {{
        label="驱动层 (Driver)"
        style="dashed,filled"
        fillcolor="#FFF3E0"
        color="{OKABE_ITO['vermillion']}"
        penwidth=2
        
        drv_uart [label="UART1-7\\n串口通信 (DMA)", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
        drv_gpio [label="GPIO\\n输入输出控制", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
        drv_tim [label="TIM7/10/14\\n定时器驱动", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
        drv_adc [label="ADC\\n模拟信号采集", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
        drv_i2c [label="Software I2C\\nEEPROM存储", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
        drv_1wire [label="1-Wire\\nRFID/DS2401", fillcolor="#FFE0B2", color="{OKABE_ITO['vermillion']}"];
    }}

    // ===== 硬件抽象层 (2个节点) =====
    subgraph cluster_hal {{
        label="硬件抽象层 (HAL)"
        style="dashed,filled"
        fillcolor="#FCE4EC"
        color="{OKABE_ITO['reddish_purple']}"
        penwidth=2
        
        hal_fill1 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        hal_fill2 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        hal_board [label="board.h/c\\n引脚与外设配置", fillcolor="#F8BBD9", color="{OKABE_ITO['reddish_purple']}"];
        hal_stm32 [label="STM32F4xx HAL\\n外设驱动库", fillcolor="#F8BBD9", color="{OKABE_ITO['reddish_purple']}"];
        hal_fill3 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        hal_fill4 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
    }}

    // ===== 硬件层 (4个节点) =====
    subgraph cluster_hw {{
        label="硬件层 (Hardware)"
        style="dashed,filled"
        fillcolor="#ECEFF1"
        color="{OKABE_ITO['black']}"
        penwidth=2
        
        hw_fill1 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        hw_mcu [label="STM32F4xx\\nCortex-M4 100MHz", fillcolor="#CFD8DC", color="{OKABE_ITO['black']}"];
        hw_uart [label="UART\\n7路串口", fillcolor="#CFD8DC", color="{OKABE_ITO['black']}"];
        hw_gpio [label="GPIO\\n多路输入输出", fillcolor="#CFD8DC", color="{OKABE_ITO['black']}"];
        hw_eeprom [label="AT24C02\\n2Kb EEPROM", fillcolor="#CFD8DC", color="{OKABE_ITO['black']}"];
        hw_fill2 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
    }}

    // ===== 外部设备层 (5个节点) =====
    subgraph cluster_peri {{
        label="外部设备 (Peripherals)"
        style="dashed,filled"
        fillcolor="#F1F8E9"
        color="{OKABE_ITO['orange']}"
        penwidth=2
        
        peri_fill1 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
        per_motor [label="无刷电机\\nA/B双通道", fillcolor="#DCEDC8", color="{OKABE_ITO['orange']}"];
        per_lcd [label="LCD显示屏\\n4.3寸触摸", fillcolor="#DCEDC8", color="{OKABE_ITO['orange']}"];
        per_pedal [label="脚踏板\\n油门+按键", fillcolor="#DCEDC8", color="{OKABE_ITO['orange']}"];
        per_hand [label="手柄\\nRFID识别", fillcolor="#DCEDC8", color="{OKABE_ITO['orange']}"];
        per_pump [label="灌注/注水泵\\n流量控制", fillcolor="#DCEDC8", color="{OKABE_ITO['orange']}"];
        peri_fill2 [label="", style="invisible", width={LAYOUT_CONFIG['filler_width']}, height=0.01];
    }}

    // ===== 连接线 (虚线) =====
    app_parser -> mw_freertos [color="{OKABE_ITO['blue']}", style=dashed];
    app_screen -> mw_apptask [color="{OKABE_ITO['blue']}", style=dashed];
    app_task -> mw_freertos [color="{OKABE_ITO['blue']}", style=dashed];
    app_task -> mw_apptask [color="{OKABE_ITO['blue']}", style=dashed];

    mw_freertos -> drv_uart [color="{OKABE_ITO['bluish_green']}", style=dashed];
    mw_freertos -> drv_gpio [color="{OKABE_ITO['bluish_green']}", style=dashed];
    mw_freertos -> drv_tim [color="{OKABE_ITO['bluish_green']}", style=dashed];
    mw_apptask -> drv_uart [color="{OKABE_ITO['bluish_green']}", style=dashed];
    mw_apptask -> drv_gpio [color="{OKABE_ITO['bluish_green']}", style=dashed];

    drv_uart -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];
    drv_gpio -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];
    drv_tim -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];
    drv_adc -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];
    drv_i2c -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];
    drv_1wire -> hal_board [color="{OKABE_ITO['vermillion']}", style=dashed];

    hal_board -> hal_stm32 [color="{OKABE_ITO['reddish_purple']}", style=dashed];
    hal_stm32 -> hw_mcu [color="{OKABE_ITO['black']}", style=dashed];
    hal_stm32 -> hw_uart [color="{OKABE_ITO['black']}", style=dashed];
    hal_stm32 -> hw_gpio [color="{OKABE_ITO['black']}", style=dashed];
    
    hw_uart -> per_motor [color="{OKABE_ITO['orange']}", style=dashed];
    hw_uart -> per_lcd [color="{OKABE_ITO['orange']}", style=dashed];
    hw_uart -> per_pedal [color="{OKABE_ITO['orange']}", style=dashed];
    hw_uart -> per_hand [color="{OKABE_ITO['orange']}", style=dashed];
    hw_gpio -> per_pump [color="{OKABE_ITO['orange']}", style=dashed];
    hw_eeprom -> hal_stm32 [color="{OKABE_ITO['black']}", style=dashed];

    // ===== 强制每层居中对齐 =====
    {{rank=same; app_fill1 app_parser app_screen app_task app_fill2}}
    {{rank=same; mw_fill1 mw_fill2 mw_freertos mw_apptask mw_fill3 mw_fill4}}
    {{rank=same; drv_uart drv_gpio drv_tim drv_adc drv_i2c drv_1wire}}
    {{rank=same; hal_fill1 hal_fill2 hal_board hal_stm32 hal_fill3 hal_fill4}}
    {{rank=same; hw_fill1 hw_mcu hw_uart hw_gpio hw_eeprom hw_fill2}}
    {{rank=same; peri_fill1 per_motor per_lcd per_pedal per_hand per_pump peri_fill2}}
}}
'''

dot_file = 'docs/assets/software_architecture.dot'
with open(dot_file, 'w', encoding='utf-8') as f:
    f.write(dot_content)

print(f"DOT 文件已生成: {dot_file}")

GRAPHVIZ_PATH = r"D:\Soft_Install_HL\graphviz\bin"
DOT_EXE = os.path.join(GRAPHVIZ_PATH, "dot.exe")

try:
    import subprocess
    png_file = 'docs/assets/software_architecture.png'
    result = subprocess.run([DOT_EXE, '-Tpng', '-Gdpi=150', '-o', png_file, dot_file], capture_output=True, text=True)
    if result.returncode == 0:
        print(f"PNG 图片已生成: {png_file}")
    else:
        print(f"Graphviz 错误: {result.stderr}")
except Exception as e:
    print(f"生成 PNG 时出错: {e}")

try:
    svg_file = 'docs/assets/software_architecture.svg'
    result = subprocess.run([DOT_EXE, '-Tsvg', '-o', svg_file, dot_file], capture_output=True, text=True)
    if result.returncode == 0:
        print(f"SVG 矢量图已生成: {svg_file}")
except Exception as e:
    print(f"生成 SVG 时出错: {e}")

print("\n=== 生成完成 ===")
