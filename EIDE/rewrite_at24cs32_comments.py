from pathlib import Path


ROOT = Path(r"D:\Study_HL\F413EXOsSSCH_RTOSV1.5")
HEADER = ROOT / "User" / "Peripheral" / "include" / "at24cs32.h"
SOURCE = ROOT / "User" / "Peripheral" / "EEPROM" / "at24cs32.c"


HEADER_TEXT = """// at24cs32.h
#ifndef __AT24CS32_H
#define __AT24CS32_H

#include "stm32f4xx_hal.h"
#include "i2c.h"

/*
 * AT24CS32 驱动说明
 * 1) 提供 EEPROM 数据区读写接口，使用 16 位字地址，并支持自动分页写入。
 * 2) 提供 128-bit 唯一序列号读取接口，便于做设备认证或信息追踪。
 * 3) 提供 I2C2 / I2C3 显式接口，避免在 RTOS 场景下切换全局句柄带来的并发风险。
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
\tAT24CS32_I2C_BUS_2 = 2,
\tAT24CS32_I2C_BUS_3 = 3
} AT24CS32_I2C_Bus;

/*
 * RTOS 推荐接口：
 * 显式绑定总线，避免多个任务复用同一套抽象接口时发生句柄混用。
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
"""


REPLACEMENTS = [
    ("/* I2C 阻塞读写超时，单位毫�?*/", "/* I2C 阻塞读写超时时间，单位毫秒 */"),
    ("/* 连续读取时的分块长度，避免单次传输过�?*/", "/* 连续读取时的分块长度，避免单次传输过长 */"),
    ("/* 发生 BUSY/TIMEOUT 后，I2C 软恢复后的等待时�?*/", "/* 发生 BUSY/TIMEOUT 后，I2C 软恢复完成后的等待时间 */"),
    ("/* 调试信息中的操作类型�? 表示读，2 表示�?*/", "/* 调试信息中的操作类型定义：1 表示读，2 表示写 */"),
    ("/* 检�?I2C 句柄是否有效 */", "/* 检查 I2C 句柄是否有效 */"),
    ("/* 封装 16 位字地址读接�?*/\n/* 封装 16 位字地址读接�?*/", "/* 封装 16 位字地址读接口 */"),
    ("/* 封装 16 位字地址写接�?*/\n/* 封装 16 位字地址写接�?*/", "/* 封装 16 位字地址写接口 */"),
    ("/* 通用序列号读取实现（按指�?I2C 句柄执行�?*/", "/* 通用序列号读取实现，按指定 I2C 句柄执行 */"),
    ("/* 序列号区固定地址�?x0800，长�?16 字节 */", "/* 序列号区固定地址为 0x0800，长度为 16 字节 */"),
    ("/* 通用写实现（按指�?I2C 句柄执行�?*/", "/* 通用写实现，按指定 I2C 句柄执行 */"),
    ("/* 本次实际写长�?*/", "/* 本次实际写入长度 */"),
    ("/* 等待 EEPROM 写周期完�?*/", "/* 等待 EEPROM 写周期完成 */"),
    ("/* 通用读实现（按指�?I2C 句柄执行�?*/", "/* 通用读实现，按指定 I2C 句柄执行 */"),
    ("/* 每次读取固定分块，提升稳定�?*/", "/* 每次读取固定分块，提升稳定性 */"),
    ("/* 计算页和校验：前30字节累加和，�?6位回�?*/", "/* 计算页和校验值：对前 30 字节做 16 位累加 */"),
    ("/* 校验页和：page[30] 高字节，page[31] 低字�?*/", "/* 校验页尾页和：page[30] 为高字节，page[31] 为低字节 */"),
    ("/* 通用按页读取：读取整页后立即做页和校�?*/", "/* 通用按页读取：读取整页后立即做页和校验 */"),
    ("/* I2C2 显式接口：按页读�?*/", "/* I2C2 显式接口：按页读取 */"),
    ("/* I2C3 显式接口：按页读�?*/", "/* I2C3 显式接口：按页读取 */"),
    ("/* I2C2 显式接口：按页写�?*/", "/* I2C2 显式接口：按页写入 */"),
    ("/* I2C3 显式接口：按页写�?*/\n/* I2C3 显式接口：按页写�?*/", "/* I2C3 显式接口：按页写入 */"),
]


BLOCK_REPLACEMENTS = [
    (
        "/*\n * 保存最近一�?EEPROM I2C 访问的完整细节�? * 这里不区分“成功调试信息”和“失败调试信息”，而是始终记录最后一次真实发起的底层访问�? * 这样一来，上层在发现读失败后，马上调用 GetLastDebugInfo() 就能看到该次失败对应的参数和 HAL 状态�? */",
        "/*\n * 保存最近一次 EEPROM I2C 访问的完整细节。\n * 这里不区分“成功调试信息”和“失败调试信息”，而是始终记录最后一次真实发起的底层访问。\n * 这样一来，上层在发现读失败后，马上调用 GetLastDebugInfo() 就能看到该次失败对应的参数和 HAL 状态。\n */",
    ),
    (
        "/*\n * 统一记录一次底�?I2C EEPROM 访问的关键参数�? * before 调用前先写入“发起前状态”，after 调用后再补齐“返回状态与错误码”，便于定位问题出在发起前还是发起后�? */",
        "/*\n * 统一记录一次底层 I2C EEPROM 访问的关键参数。\n * 调用前先写入“发起前状态”，调用后再补齐“返回状态与错误码”。\n * 这样更容易判断问题出在访问发起前，还是出在访问过程中。\n */",
    ),
    (
        "/*\n * 判断当前 I2C 句柄是否就是手柄认证所使用�?I2C2�? * 现场问题目前集中�?I2C2，所以后续恢复逻辑会优先对 I2C2 做定向处理�? */",
        "/*\n * 判断当前 I2C 句柄是否就是手柄认证所使用的 I2C2。\n * 当前现场问题主要集中在 I2C2，因此恢复逻辑会优先对 I2C2 做定向处理。\n */",
    ),
    (
        "/*\n * 在真正访�?EEPROM 前，先主动确认器件是�?ready�? * 这一步的意义是把“器件未应答/总线未空闲”和“正式读写失败”区分开�? * 方便现场根据 HSDBG 报文判断失败到底发生在访问前还是访问中�? */",
        "/*\n * 在真正访问 EEPROM 前，先主动确认器件处于 ready 状态。\n * 这样可以把“器件未应答 / 总线未空闲”和“正式读写失败”区分开。\n * 现场分析时可以借助 HSDBG 报文更快判断问题发生在访问前还是访问中。\n */",
    ),
    (
        "/*\n * 判断当前失败是否值得执行一�?I2C 软恢复�? * 这里重点关注 HAL_BUSY、HAL_TIMEOUT 以及 HAL_I2C_ERROR_TIMEOUT�? * 因为这几种情况都与当前现场看到的 BUSY Flag 超时高度一致�? */",
        "/*\n * 判断当前失败是否值得执行一次 I2C 软恢复。\n * 这里重点关注 HAL_BUSY、HAL_TIMEOUT 以及 HAL_I2C_ERROR_TIMEOUT。\n * 因为这些情况都与现场看到的 BUSY Flag 超时高度一致。\n */",
    ),
    (
        "/*\n * �?I2C 总线执行一次最小侵入的软恢复�? * 当前优先采用 HAL DeInit + Init 的方式，把外设状态机�?BUSY/TIMEOUT 中拉回来�? * 这样改动面较小，也更适合先验证是不是“外设状态卡死”导致的首包读取失败�? */",
        "/*\n * 对 I2C 总线执行一次最小侵入的软恢复。\n * 当前优先采用 HAL DeInit + Init 的方式，把外设状态机从 BUSY/TIMEOUT 状态中拉回来。\n * 这样改动面较小，也更适合先验证是不是“外设状态卡死”导致的首包读取失败。\n */",
    ),
    (
        "/*\n     * 每次发起 EEPROM 读取前，先记录“调用前�?I2C 状态”�?     * 如果后面现场再次出现 BUSY/TIMEOUT，就能直接从调试报文里看出：\n     * 是访问过程中卡住，还是发起访问之前总线就已经不干净了�?     */",
        "/*\n     * 每次发起 EEPROM 读取前，先记录“调用前的 I2C 状态”。\n     * 如果后面现场再次出现 BUSY/TIMEOUT，就能直接从调试报文里看出：\n     * 是访问过程中卡住，还是发起访问之前总线就已经不干净了。\n     */",
    ),
    (
        "/*\n     * 第一步先做设�?ready 探测，而不是直接冲�?Mem_Read�?     * 这样如果器件尚未 ready，或者总线在这里就已经 Busy�?     * 我们可以先尝试做一次恢复，避免把真正的读操作浪费掉�?     */",
        "/*\n     * 第一步先做设备 ready 探测，而不是直接调用 Mem_Read。\n     * 这样如果器件尚未 ready，或者总线在这里就已经 Busy，\n     * 可以先尝试做一次恢复，避免把真正的读操作浪费掉。\n     */",
    ),
    (
        "/*\n     * 设备 ready 后再执行真正�?EEPROM 读�?     * 如果这里仍然遇到 BUSY/TIMEOUT，再做一次软恢复和单次重试�?     * 这样既能提升现场容错，又不会因为无限重试把问题掩盖掉�?     */",
        "/*\n     * 设备 ready 后再执行真正的 EEPROM 读取。\n     * 如果这里仍然遇到 BUSY/TIMEOUT，再做一次软恢复和单次重试。\n     * 这样既能提升现场容错，又不会因为无限重试把问题掩盖掉。\n     */",
    ),
    (
        "/* 无论最终成功还是失败，都把本次访问的真实结果完整回填到调试结构里�?*/",
        "/* 无论最终成功还是失败，都把本次访问的真实结果完整回填到调试结构体里 */",
    ),
    (
        "/*\n     * 写操作沿用与读操作一致的调试记录策略�?     * 这样后续如果现场转为排查“写入失败”问题，也能直接复用同一套报文分析思路�?     */",
        "/*\n     * 写操作沿用与读操作一致的调试记录策略。\n     * 这样后续如果现场转为排查“写入失败”问题，也能直接复用同一套报文分析思路。\n     */",
    ),
    (
        "/*\n     * 先确认器件可应答，再做真正的写入�?     * �?EEPROM 来说，这一步也能避免“上一轮写周期尚未结束时又立即写入”带来的误判�?     */",
        "/*\n     * 先确认器件可应答，再做真正的写入。\n     * 对 EEPROM 来说，这一步也能避免“上一轮写周期尚未结束时又立即写入”带来的误判。\n     */",
    ),
    (
        "/* 统一回填本次写操作的底层状态，便于现场直接对照 HSDBG 分析�?*/",
        "/* 统一回填本次写操作的底层状态，便于现场直接对照 HSDBG 分析 */",
    ),
    (
        "/*\n * 清空最近一次调试信息�? * 上层在准备开始一次“关键认证读”之前先调用这个函数，后续若发生失败，就能确认读到的内容一定对应本轮访问�? */",
        "/*\n * 清空最近一次调试信息。\n * 上层在准备开始一次“关键认证读取”之前先调用这个函数，\n * 后续若发生失败，就能确认读到的内容一定对应本轮访问。\n */",
    ),
    (
        "/*\n * 读取最近一�?EEPROM I2C 访问调试信息�? * 使用值拷贝，避免上层直接改动底层保存的原始调试结果�? */",
        "/*\n * 读取最近一次 EEPROM I2C 访问调试信息。\n * 使用值拷贝，避免上层直接改动底层保存的原始调试结果。\n */",
    ),
]


def main() -> None:
    HEADER.write_text(HEADER_TEXT, encoding="utf-8")

    text = SOURCE.read_text(encoding="utf-8", errors="replace")
    for old, new in BLOCK_REPLACEMENTS:
        text = text.replace(old, new)
    for old, new in REPLACEMENTS:
        text = text.replace(old, new)
    SOURCE.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
