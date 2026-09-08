// at24cs32.h
#ifndef __AT24CS32_H
#define __AT24CS32_H

#include "stm32f4xx_hal.h"
#include "bsp_i2c_bus.h"

/*
 * AT24CS32 驱动说明
 * 1) 提供 EEPROM 数据区读写接口，使用 16 位字地址，并支持自动分页写入。
 * 2) 提供 128-bit 唯一序列号读取接口，便于做设备认证或信息追踪。
 * 3) 函数名直接区分I2C2/I2C3，调用时不用先修改一个全局“当前总线”变量。
 */

/* 用户 EEPROM 存储区设备地址，SOT23-5 封装下 A2/A1/A0 为内部下拉 */
#define AT24CS32_EEPROM_WRITE_ADDR    0xA0  // 1010 000 0 (R/W=0)
#define AT24CS32_EEPROM_READ_ADDR     0xA1  // 1010 000 1 (R/W=1)

/* 128 位唯一序列号区设备地址，只读区单独编址 */
#define AT24CS32_SN_READ_ADDR         0xB1  // 1011 000 1 (R/W=1)
#define AT24CS32_SN_WRITE_ADDR        0xB0  // 1011 000 0 (R/W=0)

/* 存储器基础参数，来源于器件规格书 */
#define AT24CS32_PAGE_SIZE            32    // 每页 32 字节
#define AT24CS32_TOTAL_SIZE           4096  // 总容量 32Kb，即 4096 Byte
#define AT24CS32_SN_SIZE              16    // 序列号长度 128-bit，即 16 Byte
#define AT24CS32_PAGE_COUNT           128   // 总页数 4096 / 32 = 128
#define AT24CS32_PAGE_DATA_SIZE       (AT24CS32_PAGE_SIZE - 2U) // 每页前 30 字节为有效数据，最后 2 字节为页和校验

/* 序列号区固定起始字地址，规格书要求从 0x0800 访问 */
#define AT24CS32_SN_WORD_ADDR_HIGH    0x08  // 高字节：0x08 (A11=1, A10=0)
#define AT24CS32_SN_WORD_ADDR_LOW     0x00  // 低字节：0x00

typedef enum {
	AT24CS32_I2C_BUS_2 = 2,
	AT24CS32_I2C_BUS_3 = 3
} AT24CS32_I2C_Bus;

/*
 * 按函数名选择I2C2或I2C3，避免访问错手柄；同一总线上的读写仍需由调用方安排顺序。
 */
/* 通过 I2C2 读取 16 字节序列号，返回 0 成功，非 0 失败 */
uint8_t AT24CS32_ReadSerialNumber_I2C2(uint8_t *sn_buf);
/* 通过 I2C3 读取 16 字节序列号，返回 0 成功，非 0 失败 */
uint8_t AT24CS32_ReadSerialNumber_I2C3(uint8_t *sn_buf);
/* 通过 I2C2 连续写数据，返回 1 成功，0 失败 */
uint8_t AT24CS32_WriteBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len);
/* 通过 I2C3 连续写数据，返回 1 成功，0 失败 */
uint8_t AT24CS32_WriteBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len);
/* 通过 I2C2 连续读数据，返回 1 成功，0 失败 */
uint8_t AT24CS32_ReadBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len);
/* 通过 I2C3 连续读数据，返回 1 成功，0 失败 */
uint8_t AT24CS32_ReadBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len);

/* 计算页和校验值：对页内前 30 字节做 16 位累加和，结果按大端方式存放到页尾 */
uint16_t AT24CS32_PageChecksum(const uint8_t *page_buf);
/* 校验页尾的页和是否与当前页数据重新计算出的结果一致，一致返回 1，否则返回 0 */
uint8_t AT24CS32_VerifyPageChecksum(const uint8_t *page_buf);
/* 通过 I2C2 按页读取 32 字节，并在读完后立即执行页和校验，成功返回 1 */
uint8_t AT24CS32_ReadPage_I2C2(uint16_t page_index, uint8_t *page_buf);
/* 通过 I2C3 按页读取 32 字节，并在读完后立即执行页和校验，成功返回 1 */
uint8_t AT24CS32_ReadPage_I2C3(uint16_t page_index, uint8_t *page_buf);
/* 通过 I2C2 按页写入 32 字节，写入前自动刷新最后 2 字节的页和校验，成功返回 1 */
uint8_t AT24CS32_WritePage_I2C2(uint16_t page_index, const uint8_t *page_buf);
/* 通过 I2C3 按页写入 32 字节，写入前自动刷新最后 2 字节的页和校验，成功返回 1 */
uint8_t AT24CS32_WritePage_I2C3(uint16_t page_index, const uint8_t *page_buf);

/*
 * 最近一次 EEPROM I2C 访问调试信息。
 * 这个结构体用于现场定位“失败发生在什么器件地址、什么内部地址、什么长度、HAL 返回了什么状态”。
 * 底层每次发起真实读写访问前后都会更新这些字段，上层在失败后立刻读取即可拿到最后一次访问细节。
 */
typedef struct
{
    uint8_t op_type;            /* 操作类型：1 表示读，2 表示写 */
    uint16_t dev_addr;          /* 本次访问使用的 8 位器件地址，例如 0xA0 / 0xB0 */
    uint16_t mem_addr;          /* EEPROM 内部 16 位字地址 */
    uint16_t data_len;          /* 本次访问的数据长度 */
    uint8_t hal_status;         /* HAL_I2C_Mem_Read/Write 返回值：0=HAL_OK，1=HAL_ERROR，2=HAL_BUSY，3=HAL_TIMEOUT */
    uint32_t hal_error;         /* HAL_I2C_GetError() 返回的错误位图 */
    uint32_t i2c_state_before;  /* 发起访问前的 I2C 状态，便于判断是否一开始就处于 Busy */
    uint32_t i2c_state_after;   /* 访问返回后的 I2C 状态，便于判断失败后总线停在什么状态 */
} AT24CS32_DebugInfo;

/* 清空最近一次 EEPROM I2C 调试信息，便于让上层明确知道“本轮失败对应的是哪次访问” */
void AT24CS32_ClearLastDebugInfo(void);
/* 读取最近一次 EEPROM I2C 调试信息，info 非空时会被完整填充 */
void AT24CS32_GetLastDebugInfo(AT24CS32_DebugInfo *info);

#endif /* __AT24CS32_H */
