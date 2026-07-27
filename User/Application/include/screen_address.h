#ifndef SCREEN_ADDRESS_H
#define SCREEN_ADDRESS_H

/* 启动页页号：上电等待期固定停留在 EX8 启动页，避免屏幕复位后沿用上一次背景页。 */
#define UIDP_LCD_PAGE_STARTUP 0U
/* 脚踏定标页页号：启动页专用入口触发后进入该页面，正常业务任务尚未创建。 */
#define UIDP_LCD_PAGE_PEDAL_CALIBRATION 3U
/* 主运行页页号：开机后强制切换到8寸屏主运行页面。 */
#define UIDP_LCD_PAGE_MAIN_RUN 4U
/* A手柄识别VP：用于刷新A通道手柄类型和在线状态图标。 */
#define UIDP_LCD_VP_HANDLE_A 0x1401U
/* B手柄识别VP：用于刷新B通道手柄类型和在线状态图标。 */
#define UIDP_LCD_VP_HANDLE_B 0x1402U
/* 开口定位VP：用于显示开口定位按钮或状态图标。 */
#define UIDP_LCD_VP_OPEN_POSITION 0x1403U
/* 刀具识别结果VP：用于显示自动识别或手动选择后的刀具结果图标。 */
#define UIDP_LCD_VP_TOOL_RESULT 0x1404U
/* 磨头按钮VP：用于显示磨头按钮的禁用、可选或选中状态。 */
#define UIDP_LCD_VP_TOOL_BURR 0x1405U
/* 刨刀按钮VP：用于显示刨刀按钮的禁用、可选或选中状态。 */
#define UIDP_LCD_VP_TOOL_BLADE 0x1406U
/* 自动识别VP：用于显示自动识别按钮的待触发状态。 */
#define UIDP_LCD_VP_AUTO_RECOGNIZE 0x1407U
/* 速度背景VP：用于显示转速区域背景和可用状态。 */
#define UIDP_LCD_VP_SPEED_AREA 0x1408U
/* 频率背景VP：用于显示频率区域背景和可用状态。 */
#define UIDP_LCD_VP_FREQ_AREA 0x1409U
/* 正转方向VP：用于显示正转方向按钮状态。 */
#define UIDP_LCD_VP_DIR_FORWARD 0x1410U
/* 往复方向VP：用于显示往复方向按钮状态。 */
#define UIDP_LCD_VP_DIR_OSC 0x1411U
/* 反转方向VP：用于显示反转方向按钮状态。 */
#define UIDP_LCD_VP_DIR_REVERSE 0x1412U
/* 脚控模式VP：用于显示脚控控制方式状态。 */
#define UIDP_LCD_VP_CONTROL_FOOT 0x1413U
/* 手控模式VP：用于显示手控控制方式状态。 */
#define UIDP_LCD_VP_CONTROL_HANDLE 0x1414U
/* 触控模式VP：用于显示触控控制方式状态。 */
#define UIDP_LCD_VP_CONTROL_TOUCH 0x1415U
/* 外部控制VP：用于显示外部控制或预值通信状态图标。 */
#define UIDP_LCD_VP_CONTROL_EXTERNAL 0x1416U
/* 屏幕安装镜像时开启 A/B 泵显示交换：1=逻辑 A 写到屏幕 B 位，0=保持原始 A/B 位置。 */
#ifndef UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE
#define UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE 0U
#endif
/* A泵类型VP：用于显示A泵抽吸、灌注或注水类型标题。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_TYPE 0x1418U /* 镜像开启：逻辑 A 泵类型写到屏幕 B 泵位置。 */
#else
#define UIDP_LCD_VP_PUMP_A_TYPE 0x1417U /* 镜像关闭：逻辑 A 泵类型写到原 A 泵位置。 */
#endif
/* B泵类型VP：用于显示B泵抽吸、灌注或注水类型标题。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_TYPE 0x1417U /* 镜像开启：逻辑 B 泵类型写到屏幕 A 泵位置。 */
#else
#define UIDP_LCD_VP_PUMP_B_TYPE 0x1418U /* 镜像关闭：逻辑 B 泵类型写到原 B 泵位置。 */
#endif
/* A泵档位区VP：用于刷新A泵蓝色流量可视化档位。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_GEAR_AREA 0x1420U /* 镜像开启：逻辑 A 泵档位写到屏幕 B 泵档位区。 */
#else
#define UIDP_LCD_VP_PUMP_A_GEAR_AREA 0x1419U /* 镜像关闭：逻辑 A 泵档位写到原 A 泵档位区。 */
#endif
/* B泵档位区VP：用于刷新B泵蓝色流量可视化档位。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_GEAR_AREA 0x1419U /* 镜像开启：逻辑 B 泵档位写到屏幕 A 泵档位区。 */
#else
#define UIDP_LCD_VP_PUMP_B_GEAR_AREA 0x1420U /* 镜像关闭：逻辑 B 泵档位写到原 B 泵档位区。 */
#endif
/* A泵加号VP：当前代码用于刷新A泵上调按钮。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_PLUS 0x1422U /* 镜像开启：逻辑 A 泵加号写到屏幕 B 泵加号位。 */
#else
#define UIDP_LCD_VP_PUMP_A_PLUS 0x1421U /* 镜像关闭：逻辑 A 泵加号写到原 A 泵加号位。 */
#endif
/* B泵启停VP：当前代码用于刷新B泵启停按钮，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_BUTTON 0x1427U /* 镜像开启：逻辑 B 泵启停按钮写到屏幕 A 泵按钮位。 */
#else
#define UIDP_LCD_VP_PUMP_B_BUTTON 0x1428U /* 镜像关闭：逻辑 B 泵启停按钮写到原 B 泵按钮位。 */
#endif
/* B泵加号VP：当前代码用于刷新B泵上调按钮，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_PLUS 0x1421U /* 镜像开启：逻辑 B 泵加号写到屏幕 A 泵加号位。 */
#else
#define UIDP_LCD_VP_PUMP_B_PLUS 0x1422U /* 镜像关闭：逻辑 B 泵加号写到原 B 泵加号位。 */
#endif
/* A泵减号VP：当前代码用于刷新A泵下调按钮，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_MINUS 0x1424U /* 镜像开启：逻辑 A 泵减号写到屏幕 B 泵减号位。 */
#else
#define UIDP_LCD_VP_PUMP_A_MINUS 0x1423U /* 镜像关闭：逻辑 A 泵减号写到原 A 泵减号位。 */
#endif
/* B泵减号VP：当前代码用于刷新B泵下调按钮，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_MINUS 0x1423U /* 镜像开启：逻辑 B 泵减号写到屏幕 A 泵减号位。 */
#else
#define UIDP_LCD_VP_PUMP_B_MINUS 0x1424U /* 镜像关闭：逻辑 B 泵减号写到原 B 泵减号位。 */
#endif
/* A泵单位VP：当前代码用于刷新A泵流量单位，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_UNIT 0x1426U /* 镜像开启：逻辑 A 泵单位写到屏幕 B 泵单位位。 */
#else
#define UIDP_LCD_VP_PUMP_A_UNIT 0x1425U /* 镜像关闭：逻辑 A 泵单位写到原 A 泵单位位。 */
#endif
/* B泵单位VP：当前代码用于刷新B泵流量单位，后续地址校正时统一从此宏调整。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_UNIT 0x1425U /* 镜像开启：逻辑 B 泵单位写到屏幕 A 泵单位位。 */
#else
#define UIDP_LCD_VP_PUMP_B_UNIT 0x1426U /* 镜像关闭：逻辑 B 泵单位写到原 B 泵单位位。 */
#endif
/* 报警提示VP：当前代码用于刷新报警提示图片，后续地址校正时统一从此宏调整。 */
#define UIDP_LCD_VP_ALARM_TIP 0x1429U
/* A泵启停旧VP：当前代码仍写入A泵启停按钮的历史地址，暂不在本轮修正。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_BUTTON 0x1428U /* 镜像开启：逻辑 A 泵启停按钮写到屏幕 B 泵按钮位。 */
#else
#define UIDP_LCD_VP_PUMP_A_BUTTON 0x1427U /* 镜像关闭：逻辑 A 泵启停按钮写到原 A 泵按钮位。 */
#endif
/* 主运行页触控工作 VP：用于 0x5520 保活触发的触控运行显示。 */
#define UIDP_LCD_VP_TOUCH_WORK 0x1430U
/* 转速数值VP：用于写入主电机转速数值。 */
#define UIDP_LCD_VP_SPEED_VALUE 0x3420U
/* 频率数值VP：用于写入往复或方向相关频率数值。 */
#define UIDP_LCD_VP_FREQ_VALUE 0x3470U
/* A泵流量数值VP：用于写入A泵当前流量数值。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_A_VALUE 0x3550U /* 镜像开启：逻辑 A 泵数值写到屏幕 B 泵数值 VP。 */
#else
#define UIDP_LCD_VP_PUMP_A_VALUE 0x3530U /* 镜像关闭：逻辑 A 泵数值写到原 A 泵数值 VP。 */
#endif
/* B泵流量数值VP：用于写入B泵当前流量数值。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_VP_PUMP_B_VALUE 0x3530U /* 镜像开启：逻辑 B 泵数值写到屏幕 A 泵数值 VP。 */
#else
#define UIDP_LCD_VP_PUMP_B_VALUE 0x3550U /* 镜像关闭：逻辑 B 泵数值写到原 B 泵数值 VP。 */
#endif
/* 脚踏标定左低值VP：用于显示左侧脚踏低位存储值。 */
#define UIDP_LCD_VP_PEDAL_LEFT_LOW_STORE 0x3740U
/* 脚踏标定左高值VP：用于显示左侧脚踏高位存储值。 */
#define UIDP_LCD_VP_PEDAL_LEFT_HIGH_STORE 0x3750U
/* 脚踏标定左中值VP：用于显示左侧脚踏中间存储值。 */
#define UIDP_LCD_VP_PEDAL_LEFT_MID_STORE 0x3790U
/* 脚踏标定右低值VP：用于显示右侧脚踏低位存储值。 */
#define UIDP_LCD_VP_PEDAL_RIGHT_LOW_STORE 0x3770U
/* 脚踏标定右高值VP：用于显示右侧脚踏高位存储值。 */
#define UIDP_LCD_VP_PEDAL_RIGHT_HIGH_STORE 0x3780U
/* 脚踏标定右中值VP：用于显示右侧脚踏中间存储值。 */
#define UIDP_LCD_VP_PEDAL_RIGHT_MID_STORE 0x37A0U
/* 脚踏标定左实时AD值VP：用于显示左侧脚踏实时采样值。 */
#define UIDP_LCD_VP_PEDAL_LEFT_AD_VALUE 0x3730U
/* 脚踏标定右实时AD值VP：用于显示右侧脚踏实时采样值。 */
#define UIDP_LCD_VP_PEDAL_RIGHT_AD_VALUE 0x3760U
/* 脚踏标定低位系数VP：用于显示标定页面底部低位系数。 */
#define UIDP_LCD_VP_PEDAL_LOW_KEY_VALUE 0x3700U
/* 脚踏标定中位系数VP：用于显示标定页面底部中位系数。 */
#define UIDP_LCD_VP_PEDAL_MID_KEY_VALUE 0x3710U
/* 脚踏标定高位系数VP：用于显示标定页面底部高位系数。 */
#define UIDP_LCD_VP_PEDAL_HIGH_KEY_VALUE 0x3720U
/* 刀具规格文本VP：用于写入刀具长度、直径和角度文本。 */
#define UIDP_LCD_VP_TOOL_SPEC_TEXT 0x4200U
/* 刀具规格颜色SP：用于切换刀具规格文本颜色控制字。 */
#define UIDP_LCD_SP_TOOL_SPEC_COLOR 0x8008U
/* 转速数值显示SP：用于绑定或隐藏转速数值控件。 */
#define UIDP_LCD_SP_SPEED_VALUE 0x9420U
/* 转速单位颜色SP：用于切换转速单位颜色控制字。 */
#define UIDP_LCD_SP_SPEED_UNIT_COLOR 0x9423U
/* 频率数值显示SP：用于绑定或隐藏频率数值控件。 */
#define UIDP_LCD_SP_FREQ_VALUE 0x9470U
/* A泵流量数值显示SP：用于绑定或隐藏A泵流量数值控件。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_SP_PUMP_A_VALUE 0x9550U /* 镜像开启：逻辑 A 泵数值控件绑定到屏幕 B 泵控件。 */
#else
#define UIDP_LCD_SP_PUMP_A_VALUE 0x9530U /* 镜像关闭：逻辑 A 泵数值控件保持原 A 泵控件。 */
#endif
/* A泵输出颜色SP：旧泵直连路径用于切换A泵数值颜色。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR 0x9553U /* 镜像开启：逻辑 A 泵输出颜色写到屏幕 B 泵颜色控件。 */
#else
#define UIDP_LCD_SP_PUMP_A_OUTPUT_COLOR 0x9533U /* 镜像关闭：逻辑 A 泵输出颜色保持原 A 泵颜色控件。 */
#endif
/* B泵流量数值显示SP：用于绑定或隐藏B泵流量数值控件。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_SP_PUMP_B_VALUE 0x9530U /* 镜像开启：逻辑 B 泵数值控件绑定到屏幕 A 泵控件。 */
#else
#define UIDP_LCD_SP_PUMP_B_VALUE 0x9550U /* 镜像关闭：逻辑 B 泵数值控件保持原 B 泵控件。 */
#endif
/* B泵输出颜色SP：旧泵直连路径用于切换B泵数值颜色。 */
#if (UIDP_PUMP_DISPLAY_AB_MIRROR_SWAP_ENABLE == 1U)
#define UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR 0x9533U /* 镜像开启：逻辑 B 泵输出颜色写到屏幕 A 泵颜色控件。 */
#else
#define UIDP_LCD_SP_PUMP_B_OUTPUT_COLOR 0x9553U /* 镜像关闭：逻辑 B 泵输出颜色保持原 B 泵颜色控件。 */
#endif

/* 旧屏A手柄缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_HANDLE_A_CACHE 0x1500U
/* 旧屏B手柄缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_HANDLE_B_CACHE 0x1501U
/* 旧屏A泵区域缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_PUMP_A_CACHE 0x1502U
/* 旧屏B泵区域缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_PUMP_B_CACHE 0x1506U
/* 旧屏转速栏缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_SPEED_AREA_CACHE 0x1600U
/* 旧屏频率或档位栏缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_FREQ_GEAR_CACHE 0x1601U
/* 旧屏方向组合缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_DIRECTION_GROUP_CACHE 0x1602U
/* 旧屏往复角度缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_OSC_ANGLE_CACHE 0x1606U
/* 旧屏正转方向缓存VP：已停用分支保留宏名，方便后续追溯历史地址。 */
#define UIDP_LCD_LEGACY_VP_DIR_FORWARD_CACHE 0x1310U
/* 旧屏往复方向缓存VP：已停用分支保留宏名，方便后续追溯历史地址。 */
#define UIDP_LCD_LEGACY_VP_DIR_OSC_CACHE 0x1311U
/* 旧屏脚控模式缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_CONTROL_FOOT_CACHE 0x1312U
/* 旧屏手控模式缓存VP：LCD底层图片状态缓存使用，保留历史地址语义不参与本轮新屏校正。 */
#define UIDP_LCD_LEGACY_VP_CONTROL_HANDLE_CACHE 0x1313U
/* 旧屏反转方向缓存VP：已停用分支保留宏名，方便后续追溯历史地址。 */
#define UIDP_LCD_LEGACY_VP_DIR_REVERSE_CACHE 0x1316U

#endif
