/**
 * @file board.h
 * @brief 板上引脚和硬件参数定义。
 *        改硬件时还要核对 board_profile.h 的通道交换及 Src 中实际使用的初始化代码。
 */
#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*============================================================================
 * 第一部分：串口通信接口配置 (UART)
 *============================================================================*/

/* 串口信息表，波特率单位为bit/s；实际启动配置还需核对Src/usart.c，不能只改这张表。 */
#define BOARD_UART_LIST(XX) \
    XX(1, USART1, 9600)     /* USART1 - 手柄电机驱动板 */ \
    XX(2, USART2, 9600)     /* USART2 - 外部上位机 */ \
    XX(3, USART3, 115200)   /* USART3 - RFID读卡器 */ \
    XX(4, UART4, 115200)    /* UART4 - 脚踏板 */ \
    XX(5, UART5, 9600)       /* UART5 - A泵步进驱动板 */ \
    XX(6, USART6, 115200)   /* USART6 - 显示屏 */ \
    XX(7, UART7, 9600)       /* UART7 - B泵步进驱动板 */ \
    XX(8, UART8, 115200)     /* UART8 - B通道KSZ手柄通信 */ \
    XX(10, UART10, 115200)   /* UART10 - A通道KSZ手柄通信 */

/* UART引脚配置 - 如需更改引脚，修改以下定义 */
#define BOARD_UART1_TX_PORT   GPIOB        /* USART1_TX - PB6 */
#define BOARD_UART1_TX_PIN    GPIO_PIN_6
#define BOARD_UART1_TX_AF     GPIO_AF7_USART1
#define BOARD_UART1_RX_PORT   GPIOB        /* USART1_RX - PB7 */
#define BOARD_UART1_RX_PIN    GPIO_PIN_7
#define BOARD_UART1_RX_AF     GPIO_AF7_USART1

#define BOARD_UART2_TX_PORT   GPIOD        /* USART2_TX - PD5 */
#define BOARD_UART2_TX_PIN    GPIO_PIN_5
#define BOARD_UART2_TX_AF     GPIO_AF7_USART2
#define BOARD_UART2_RX_PORT   GPIOD        /* USART2_RX - PD6 */
#define BOARD_UART2_RX_PIN    GPIO_PIN_6
#define BOARD_UART2_RX_AF     GPIO_AF7_USART2
/* V4.0 主控板使用 PD4 同时驱动 CA-IS3092W 的 DE 和 /RE，低电平接收、高电平发送。 */
#define BOARD_UART2_RS485_DIR_PORT  GPIOD        /* USART2_RD - PD4 */
#define BOARD_UART2_RS485_DIR_PIN   GPIO_PIN_4

#define BOARD_UART3_TX_PORT   GPIOD        /* USART3_TX - PD8 */
#define BOARD_UART3_TX_PIN    GPIO_PIN_8
#define BOARD_UART3_TX_AF     GPIO_AF7_USART3
#define BOARD_UART3_RX_PORT   GPIOD        /* USART3_RX - PD9 */
#define BOARD_UART3_RX_PIN    GPIO_PIN_9
#define BOARD_UART3_RX_AF     GPIO_AF7_USART3

#define BOARD_UART4_TX_PORT   GPIOA        /* UART4_TX - PA11 */
#define BOARD_UART4_TX_PIN    GPIO_PIN_11
#define BOARD_UART4_TX_AF     GPIO_AF11_UART4
#define BOARD_UART4_RX_PORT   GPIOA        /* UART4_RX - PA12 */
#define BOARD_UART4_RX_PIN    GPIO_PIN_12
#define BOARD_UART4_RX_AF     GPIO_AF11_UART4

#define BOARD_UART5_TX_PORT   GPIOB        /* UART5_TX - PB8 */
#define BOARD_UART5_TX_PIN    GPIO_PIN_8
#define BOARD_UART5_TX_AF     GPIO_AF11_UART5
#define BOARD_UART5_RX_PORT   GPIOB        /* UART5_RX - PB9 */
#define BOARD_UART5_RX_PIN    GPIO_PIN_9
#define BOARD_UART5_RX_AF     GPIO_AF11_UART5

#define BOARD_UART6_TX_PORT   GPIOC        /* USART6_TX - PC6 */
#define BOARD_UART6_TX_PIN    GPIO_PIN_6
#define BOARD_UART6_TX_AF     GPIO_AF8_USART6
#define BOARD_UART6_RX_PORT   GPIOC        /* USART6_RX - PC7 */
#define BOARD_UART6_RX_PIN    GPIO_PIN_7
#define BOARD_UART6_RX_AF     GPIO_AF8_USART6

#define BOARD_UART7_TX_PORT   GPIOE        /* UART7_TX - PE7 */
#define BOARD_UART7_TX_PIN    GPIO_PIN_7
#define BOARD_UART7_TX_AF     GPIO_AF8_UART7
#define BOARD_UART7_RX_PORT   GPIOE        /* UART7_RX - PE8 */
#define BOARD_UART7_RX_PIN    GPIO_PIN_8
#define BOARD_UART7_RX_AF     GPIO_AF8_UART7

#define BOARD_UART8_TX_PORT   GPIOE        /* UART8_TX - PE1 */
#define BOARD_UART8_TX_PIN    GPIO_PIN_1
#define BOARD_UART8_TX_AF     GPIO_AF8_UART8
#define BOARD_UART8_RX_PORT   GPIOE        /* UART8_RX/普通B实体键 - PE0 */
#define BOARD_UART8_RX_PIN    GPIO_PIN_0
#define BOARD_UART8_RX_AF     GPIO_AF8_UART8

#define BOARD_UART9_TX_PORT   GPIOD        /* UART9_TX - PD15，原物理B侧RFID发送脚。 */
#define BOARD_UART9_TX_PIN    GPIO_PIN_15    /* UART9_TX 实际对应 PD15，按 STM32F413 引脚复用表修正发送脚。 */
#define BOARD_UART9_TX_AF     GPIO_AF11_UART9
/* UART9 引脚方向以 STM32F413 复用表为准：TX=PD15，RX=PD14。 */
#define BOARD_UART9_RX_PORT   GPIOD        /* UART9_RX - PD14，原物理B侧RFID接收脚。 */
#define BOARD_UART9_RX_PIN    GPIO_PIN_14    /* UART9_RX 实际对应 PD14，按 STM32F413 引脚复用表修正接收脚。 */
#define BOARD_UART9_RX_AF     GPIO_AF11_UART9

#define BOARD_UART10_TX_PORT  GPIOE        /* UART10_TX - PE3 */
#define BOARD_UART10_TX_PIN   GPIO_PIN_3
#define BOARD_UART10_TX_AF    GPIO_AF11_UART10
#define BOARD_UART10_RX_PORT  GPIOE        /* UART10_RX/普通A实体键 - PE2 */
#define BOARD_UART10_RX_PIN   GPIO_PIN_2
#define BOARD_UART10_RX_AF    GPIO_AF11_UART10

/* UART时钟使能宏 */
#define BOARD_UART1_CLK_ENABLE()   __HAL_RCC_USART1_CLK_ENABLE()
#define BOARD_UART2_CLK_ENABLE()   __HAL_RCC_USART2_CLK_ENABLE()
#define BOARD_UART3_CLK_ENABLE()   __HAL_RCC_USART3_CLK_ENABLE()
#define BOARD_UART4_CLK_ENABLE()   __HAL_RCC_UART4_CLK_ENABLE()
#define BOARD_UART5_CLK_ENABLE()   __HAL_RCC_UART5_CLK_ENABLE()
#define BOARD_UART6_CLK_ENABLE()   __HAL_RCC_USART6_CLK_ENABLE()
#define BOARD_UART7_CLK_ENABLE()   __HAL_RCC_UART7_CLK_ENABLE()
#define BOARD_UART8_CLK_ENABLE()   __HAL_RCC_UART8_CLK_ENABLE()
#define BOARD_UART9_CLK_ENABLE()   __HAL_RCC_UART9_CLK_ENABLE()
#define BOARD_UART10_CLK_ENABLE()  __HAL_RCC_UART10_CLK_ENABLE()

/* UART DMA通道配置 */
#define BOARD_UART1_DMA_RX       DMA2_Stream2
#define BOARD_UART1_DMA_CHANNEL  DMA_CHANNEL_4
#define BOARD_UART2_DMA_RX       DMA1_Stream5
#define BOARD_UART2_DMA_CHANNEL  DMA_CHANNEL_4
#define BOARD_UART3_DMA_RX       DMA1_Stream1
#define BOARD_UART3_DMA_CHANNEL  DMA_CHANNEL_4
#define BOARD_UART4_DMA_RX       DMA1_Stream2
#define BOARD_UART4_DMA_CHANNEL  DMA_CHANNEL_4
#define BOARD_UART5_DMA_RX       DMA1_Stream0
#define BOARD_UART5_DMA_CHANNEL  DMA_CHANNEL_4
#define BOARD_UART6_DMA_RX       DMA2_Stream1
#define BOARD_UART6_DMA_CHANNEL  DMA_CHANNEL_5
#define BOARD_UART7_DMA_RX       DMA1_Stream3
#define BOARD_UART7_DMA_CHANNEL  DMA_CHANNEL_5
#define BOARD_UART8_DMA_RX       DMA1_Stream6
#define BOARD_UART8_DMA_CHANNEL  DMA_CHANNEL_5
#define BOARD_UART9_DMA_RX       DMA2_Stream7
#define BOARD_UART9_DMA_CHANNEL  DMA_CHANNEL_0
#define BOARD_UART10_DMA_RX      DMA2_Stream0
#define BOARD_UART10_DMA_CHANNEL DMA_CHANNEL_5

/*============================================================================
 * 第二部分：ADC模拟输入配置
 *============================================================================*/

/* ADC外设定义 */
#define BOARD_ADC1             hadc1

/* ADC通道定义 - 如需更改ADC通道，修改以下定义 */
#define BOARD_ADC_CHANNEL_CNT     4       /* ADC通道数量 */

/* ADC通道对应引脚和功能 */
#define BOARD_ADC_CH1_PORT        GPIOA   /* PA1 - 手柄按键1 */
#define BOARD_ADC_CH1_PIN         GPIO_PIN_1
#define BOARD_ADC_CH1_FUNCTION    "H_KEY1"   /* 手柄按键1 */

#define BOARD_ADC_CH2_PORT        GPIOA   /* PA2 - 手柄按键2 */
#define BOARD_ADC_CH2_PIN         GPIO_PIN_2
#define BOARD_ADC_CH2_FUNCTION    "H_KEY2"   /* 手柄按键2 */

#define BOARD_ADC_CH3_PORT        GPIOA   /* PA3 - 电压检测 */
#define BOARD_ADC_CH3_PIN         GPIO_PIN_3
#define BOARD_ADC_CH3_FUNCTION    "Voltage"  /* 电压检测 */

#define BOARD_ADC_CH4_PORT        GPIOA   /* PA4 - 温度传感器 */
#define BOARD_ADC_CH4_PIN         GPIO_PIN_4
#define BOARD_ADC_CH4_FUNCTION    "Temperature" /* 温度传感器 */

/* ADC通道号定义 - 对应STM32 ADC寄存器通道号 */
#define BOARD_ADC_CH1_ADC_CH      ADC_CHANNEL_1      /* ADC1_IN1 */
#define BOARD_ADC_CH2_ADC_CH      ADC_CHANNEL_2      /* ADC1_IN2 */
#define BOARD_ADC_CH3_ADC_CH      ADC_CHANNEL_3      /* ADC1_IN3 */
#define BOARD_ADC_CH4_ADC_CH      ADC_CHANNEL_4      /* ADC1_IN4 */

/*============================================================================
 * 第三部分：GPIO输入输出配置
 *============================================================================*/

/*----------------- 状态指示LED -----------------*/
#define BOARD_STATE_LED_PORT      GPIOE
#define BOARD_STATE_LED_PIN       GPIO_PIN_10

/*----------------- 蜂鸣器 ----------------------*/
#define BOARD_BEEP_PORT            GPIOA
#define BOARD_BEEP_PIN             GPIO_PIN_6

/*----------------- 手柄数据输入 -----------------*/
#define BOARD_M_D1_PORT            GPIOD
#define BOARD_M_D1_PIN             GPIO_PIN_2
#define BOARD_M_D2_PORT            GPIOD
#define BOARD_M_D2_PIN             GPIO_PIN_3
/* V4.0 主控板已取消 M_D3 输入，原 PD4 固定改作 USART2 RS485 收发方向控制。 */

/*----------------- 手柄按键输入 -----------------*/
#define BOARD_H_MD1_PORT           GPIOD
#define BOARD_H_MD1_PIN            GPIO_PIN_0
#define BOARD_H_MD2_PORT           GPIOD
#define BOARD_H_MD2_PIN            GPIO_PIN_1
#define BOARD_H_MD3_PORT           GPIOC
#define BOARD_H_MD3_PIN            GPIO_PIN_12

/*----------------- LED指示灯 -------------------*/
#define BOARD_LED_H1_PORT          GPIOC
#define BOARD_LED_H1_PIN           GPIO_PIN_8
#define BOARD_LED_H2_PORT          GPIOC
#define BOARD_LED_H2_PIN           GPIO_PIN_9
#define BOARD_LED_H3_PORT          GPIOA
#define BOARD_LED_H3_PIN           GPIO_PIN_8
#define BOARD_LED_H4_PORT          GPIOA
#define BOARD_LED_H4_PIN           GPIO_PIN_9

/*----------------- 继电器控制 -------------------*/
#define BOARD_HAS_K1K2              0U /* 0=板上无K1/K2，相关输出宏不操作引脚；1=启用继电器引脚，仅按硬件配置修改。 */
#define BOARD_K1_PORT              GPIOD
#define BOARD_K1_PIN               GPIO_PIN_14
#define BOARD_K2_PORT              GPIOD
#define BOARD_K2_PIN               GPIO_PIN_15

/*----------------- R200-K8控制 -----------------*/
#define BOARD_R200_K8_PORT         GPIOB
#define BOARD_R200_K8_PIN          GPIO_PIN_15

/*----------------- 外部中断输入 -----------------*/
#define BOARD_EXTI1_PORT           GPIOD
#define BOARD_EXTI1_PIN            GPIO_PIN_1
#define BOARD_EXTI10_PORT          GPIOC
#define BOARD_EXTI10_PIN           GPIO_PIN_10

/*============================================================================
 * 第四部分：I2C通信接口配置
 *============================================================================*/

/* 软件模拟I2C引脚配置 */
#define BOARD_IIC_SCL_PORT         GPIOC        /* I2C时钟线 - PC4 */
#define BOARD_IIC_SCL_PIN          GPIO_PIN_4
#define BOARD_IIC_SDA_PORT         GPIOC        /* I2C数据线 - PC5 */
#define BOARD_IIC_SDA_PIN          GPIO_PIN_5

/* 软件模拟I2C GPIO操作宏定义 */
#define BOARD_IIC_SCL_H()          (BOARD_IIC_SCL_PORT->BSRR = BOARD_IIC_SCL_PIN)
#define BOARD_IIC_SCL_L()          (BOARD_IIC_SCL_PORT->BSRR = ((uint32_t)BOARD_IIC_SCL_PIN << 16U))
#define BOARD_IIC_SDA_H()          (BOARD_IIC_SDA_PORT->BSRR = BOARD_IIC_SDA_PIN)
#define BOARD_IIC_SDA_L()          (BOARD_IIC_SDA_PORT->BSRR = ((uint32_t)BOARD_IIC_SDA_PIN << 16U))
#define BOARD_IIC_SDA_READ         (BOARD_IIC_SDA_PORT->IDR & BOARD_IIC_SDA_PIN)

/* I2C EEPROM地址定义 */
#define BOARD_AT24C02_ADDR         0xA0        /* AT24C02 I2C地址 */

/*============================================================================
 * 第五部分：1-Wire单总线配置
 *============================================================================*/

/* 1-Wire I 引脚配置 - 如需更改引脚，修改以下定义 */
#define BOARD_ONEWIRE_I_PORT        GPIOD       /* 1-Wire I - PD13 */
#define BOARD_ONEWIRE_I_PIN         GPIO_PIN_13
#define BOARD_ONEWIRE_I_H()         (BOARD_ONEWIRE_I_PORT->BSRR = BOARD_ONEWIRE_I_PIN)
#define BOARD_ONEWIRE_I_L()         (BOARD_ONEWIRE_I_PORT->BSRR = ((uint32_t)BOARD_ONEWIRE_I_PIN << 16U))
#define BOARD_ONEWIRE_I_READ        HAL_GPIO_ReadPin(BOARD_ONEWIRE_I_PORT, BOARD_ONEWIRE_I_PIN)

/* 1-Wire II 引脚配置 */
#define BOARD_ONEWIRE_II_PORT       GPIOD       /* 1-Wire II - PD12 */
#define BOARD_ONEWIRE_II_PIN        GPIO_PIN_12
#define BOARD_ONEWIRE_II_H()        (BOARD_ONEWIRE_II_PORT->BSRR = BOARD_ONEWIRE_II_PIN)
#define BOARD_ONEWIRE_II_L()        (BOARD_ONEWIRE_II_PORT->BSRR = ((uint32_t)BOARD_ONEWIRE_II_PIN << 16U))
#define BOARD_ONEWIRE_II_READ       HAL_GPIO_ReadPin(BOARD_ONEWIRE_II_PORT, BOARD_ONEWIRE_II_PIN)

/*============================================================================
 * 第六部分：定时器配置
 *============================================================================*/

/* 定时器外设定义 */
#define BOARD_TIM7                 htim7
#define BOARD_TIM10                htim10
#define BOARD_TIM14                htim14

/* 定时器配置参数 */
#define BOARD_TIM7_PRESCALER       100-1        /* 定时器7预分频 */
#define BOARD_TIM10_PRESCALER      50000-1     /* 定时器10预分频 - 蜂鸣器 */
#define BOARD_TIM10_PERIOD         5            /* 定时器10周期 */
#define BOARD_TIM14_PRESCALER      50000-1     /* 定时器14预分频 - 系统计时 */
#define BOARD_TIM14_PERIOD         500          /* 保留的旧重装载值，当前未引用；实际值在Src/tim.c设置，单改本宏不会改变TIM14周期。 */

/*============================================================================
 * 第七部分：系统时钟配置
 *============================================================================*/

/* 系统时钟参数 */
#define BOARD_HSE_VALUE            8000000      /* 外部晶振频率 8MHz */
#define BOARD_SYSCLK_FREQ         100000000    /* 系统时钟频率 100MHz */
#define BOARD_AHB_CLK              100000000    /* AHB时钟 */
#define BOARD_APB1_CLK            50000000     /* APB1时钟 */
#define BOARD_APB2_CLK            100000000    /* APB2时钟 */

/* PLL配置 */
#define BOARD_PLL_M               8             /* PLL预分频 */
#define BOARD_PLL_N               200           /* PLL倍频 */
#define BOARD_PLL_P               2             /* PLL分频 */
#define BOARD_PLL_Q               2             /* PLL Q分频 */
#define BOARD_PLL_R               2             /* PLL R分频 */

/*============================================================================
 * 兼容层 - 保持原有API
 *============================================================================*/

/*----------------- SYS.LED -----------------*/
#define STATELED_Pin       BOARD_STATE_LED_PIN
#define STATELED_GPIO_Port BOARD_STATE_LED_PORT

#define STATELED_ON()      (STATELED_GPIO_Port->BSRR = STATELED_Pin)
#define STATELED_OFF()     (STATELED_GPIO_Port->BSRR = ((uint32_t)STATELED_Pin << 16))

/*----------------- BUZZ/BEEP -----------------*/
#define BEEP_Pin       BOARD_BEEP_PIN
#define BEEP_GPIO_Port BOARD_BEEP_PORT

#define BEEP_ON()      (BEEP_GPIO_Port->BSRR = BEEP_Pin)
#define BEEP_OFF()     (BEEP_GPIO_Port->BSRR = ((uint32_t)BEEP_Pin << 16))

/*----------------- LED -----------------*/
#define LED_H1_Pin     BOARD_LED_H1_PIN
#define LED_H2_Pin     BOARD_LED_H2_PIN
#define LED_H12_Port  BOARD_LED_H1_PORT

#define LED_H3_Pin     BOARD_LED_H3_PIN
#define LED_H4_Pin     BOARD_LED_H4_PIN
#define LED_H34_Port  BOARD_LED_H3_PORT

#define LED_H1_ON()    (LED_H12_Port->BSRR = ((uint32_t)LED_H1_Pin << 16))
#define LED_H1_OFF()   (LED_H12_Port->BSRR = LED_H1_Pin)

#define LED_H2_ON()    (LED_H12_Port->BSRR = ((uint32_t)LED_H2_Pin << 16))
#define LED_H2_OFF()   (LED_H12_Port->BSRR = LED_H2_Pin)

#define LED_H3_ON()    (LED_H34_Port->BSRR = ((uint32_t)LED_H3_Pin << 16))
#define LED_H3_OFF()   (LED_H34_Port->BSRR = LED_H3_Pin)

#define LED_H4_ON()    (LED_H34_Port->BSRR = ((uint32_t)LED_H4_Pin << 16))
#define LED_H4_OFF()   (LED_H34_Port->BSRR = LED_H4_Pin)

/*----------------- R200-K8 -----------------*/
#define R200_K8_Pin       BOARD_R200_K8_PIN
#define R200_K8_Port      BOARD_R200_K8_PORT

#define R200_K8_5ON()    (R200_K8_Port->BSRR = ((uint32_t)R200_K8_Pin << 16))
#define R200_K8_2ON()    (R200_K8_Port->BSRR = R200_K8_Pin)

#define R200_K8_SELECT_A() (R200_K8_Port->BSRR = R200_K8_Pin) /* R200-K8 高电平选择 A 通道，供 RFID 模拟开关按业务通道切换。 */
#define R200_K8_SELECT_B() (R200_K8_Port->BSRR = ((uint32_t)R200_K8_Pin << 16U)) /* R200-K8 低电平选择 B 通道，供 RFID 模拟开关按业务通道切换。 */

/*----------------- IIC AT24C02 -----------------*/
#define IIC_SCL_Pin       BOARD_IIC_SCL_PIN
#define IIC_SDA_Pin       BOARD_IIC_SDA_PIN
#define IIC_GPIO_Prot     BOARD_IIC_SCL_PORT

/*----------------- 1-Wire DS2401 -----------------*/
#define ONE_WIRE_PinI         BOARD_ONEWIRE_I_PIN
#define ONE_WIRE_GPIO_ProtI   BOARD_ONEWIRE_I_PORT

#define ONE_WIRE_PinII        BOARD_ONEWIRE_II_PIN
#define ONE_WIRE_GPIO_ProtII  BOARD_ONEWIRE_II_PORT

/*----------------- Interface.Handle -----------------*/
#define M_D1_Pin       BOARD_M_D1_PIN
#define M_D2_Pin       BOARD_M_D2_PIN
#define M_D_GPIO_Port  BOARD_M_D1_PORT

#define M_D1_STATUS()  HAL_GPIO_ReadPin(M_D_GPIO_Port, M_D1_Pin)
#define M_D2_STATUS()  HAL_GPIO_ReadPin(M_D_GPIO_Port, M_D2_Pin)

#define H_MD1_Pin       BOARD_H_MD1_PIN
#define H_MD1_GPIO_Port BOARD_H_MD1_PORT

#define H_MD2_Pin       BOARD_H_MD2_PIN
#define H_MD2_GPIO_Port BOARD_H_MD2_PORT

#define H_MD3_Pin       BOARD_H_MD3_PIN
#define H_MD3_GPIO_Port BOARD_H_MD3_PORT

#define H_MD1_STATUS()  HAL_GPIO_ReadPin(H_MD1_GPIO_Port, H_MD1_Pin)
#define H_MD2_STATUS()  HAL_GPIO_ReadPin(H_MD2_GPIO_Port, H_MD2_Pin)
#define H_MD3_STATUS()  HAL_GPIO_ReadPin(H_MD3_GPIO_Port, H_MD3_Pin)

#define K1_Pin       BOARD_K1_PIN
#define K2_Pin       BOARD_K2_PIN
#define K1K2_GPIO_Port BOARD_K1_PORT

#if (BOARD_HAS_K1K2 == 1U)
#define K1_OFF()     (K1K2_GPIO_Port->BSRR = ((uint32_t)K1_Pin << 16))
#define K1_ON()      (K1K2_GPIO_Port->BSRR = K1_Pin)

#define K2_OFF()     (K1K2_GPIO_Port->BSRR = ((uint32_t)K2_Pin << 16))
#define K2_ON()      (K1K2_GPIO_Port->BSRR = K2_Pin)
#else
/* 新板不带 K1/K2，保留旧接口但不执行硬件动作。 */
#define K1_OFF()     ((void)0)
#define K1_ON()      ((void)0)
#define K2_OFF()     ((void)0)
#define K2_ON()      ((void)0)
#endif

/*============================================================================
 * 函数声明
 *============================================================================*/

/* GPIO初始化 */
void Board_GPIOConfiguration(void);

/*============================================================================
 * 便捷操作宏
 *============================================================================*/

/* LED操作 */
#define BOARD_LED_ON(port, pin)       HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define BOARD_LED_OFF(port, pin)      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define BOARD_LED_TOGGLE(port, pin)  HAL_GPIO_TogglePin(port, pin)

/* 蜂鸣器操作 */
#define BOARD_BEEP_ON()   HAL_GPIO_WritePin(BOARD_BEEP_PORT, BOARD_BEEP_PIN, GPIO_PIN_SET)
#define BOARD_BEEP_OFF()  HAL_GPIO_WritePin(BOARD_BEEP_PORT, BOARD_BEEP_PIN, GPIO_PIN_RESET)

/* 继电器操作 */
#if (BOARD_HAS_K1K2 == 1U)
#define BOARD_K1_ON()   HAL_GPIO_WritePin(BOARD_K1_PORT, BOARD_K1_PIN, GPIO_PIN_RESET)
#define BOARD_K1_OFF()  HAL_GPIO_WritePin(BOARD_K1_PORT, BOARD_K1_PIN, GPIO_PIN_SET)
#define BOARD_K2_ON()   HAL_GPIO_WritePin(BOARD_K2_PORT, BOARD_K2_PIN, GPIO_PIN_RESET)
#define BOARD_K2_OFF()  HAL_GPIO_WritePin(BOARD_K2_PORT, BOARD_K2_PIN, GPIO_PIN_SET)
#else
#define BOARD_K1_ON()   ((void)0)
#define BOARD_K1_OFF()  ((void)0)
#define BOARD_K2_ON()   ((void)0)
#define BOARD_K2_OFF()  ((void)0)
#endif

/* GPIO输入读取 */
#define BOARD_GPIO_READ(port, pin)  HAL_GPIO_ReadPin(port, pin)

/* 状态LED操作 */
#define BOARD_STATE_LED_ON()     HAL_GPIO_WritePin(BOARD_STATE_LED_PORT, BOARD_STATE_LED_PIN, GPIO_PIN_RESET)
#define BOARD_STATE_LED_OFF()    HAL_GPIO_WritePin(BOARD_STATE_LED_PORT, BOARD_STATE_LED_PIN, GPIO_PIN_SET)
#define BOARD_STATE_LED_TOGGLE() HAL_GPIO_TogglePin(BOARD_STATE_LED_PORT, BOARD_STATE_LED_PIN)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
