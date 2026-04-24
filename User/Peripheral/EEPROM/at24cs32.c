#include "at24cs32.h"
#include <string.h>

/* I2C 阻塞读写超时时间，单位毫秒 */
#define AT24CS32_I2C_TIMEOUT_MS        100U
/* 连续读取时的分块长度，避免单次传输过长 */
#define AT24CS32_READ_CHUNK_SIZE       64U
/* EEPROM 读前设备就绪检查的轮询次数 */
#define AT24CS32_READY_TRIALS          3U
/* 发生 BUSY/TIMEOUT 后，I2C 软恢复完成后的等待时间 */
#define AT24CS32_RECOVER_DELAY_MS      2U
/* 调试信息中的操作类型定义：1 表示读，2 表示写 */
#define AT24CS32_DEBUG_OP_READ         1U
#define AT24CS32_DEBUG_OP_WRITE        2U

/*
 * 保存最近一次 EEPROM I2C 访问的完整细节。
 * 这里不区分“成功调试信息”和“失败调试信息”，而是始终记录最后一次真实发起的底层访问。
 * 这样一来，上层在发现读失败后，马上调用 GetLastDebugInfo() 就能看到该次失败对应的参数和 HAL 状态。
 */
static AT24CS32_DebugInfo s_at24cs32_last_debug = {0U};

/*
 * 统一记录一次底层 I2C EEPROM 访问的关键参数。
 * 调用前先写入“发起前状态”，调用后再补齐“返回状态与错误码”。
 * 这样更容易判断问题出在访问发起前，还是出在访问过程中。
 */
static void AT24CS32_UpdateDebugInfo(I2C_HandleTypeDef *hi2c,
                                     uint8_t op_type,
                                     uint16_t dev_addr,
                                     uint16_t mem_addr,
                                     uint16_t data_len,
                                     HAL_StatusTypeDef hal_status)
{
    s_at24cs32_last_debug.op_type = op_type;
    s_at24cs32_last_debug.dev_addr = dev_addr;
    s_at24cs32_last_debug.mem_addr = mem_addr;
    s_at24cs32_last_debug.data_len = data_len;
    s_at24cs32_last_debug.hal_status = (uint8_t)hal_status;
    s_at24cs32_last_debug.hal_error = (hi2c != NULL) ? HAL_I2C_GetError(hi2c) : 0U;
    s_at24cs32_last_debug.i2c_state_after = (hi2c != NULL) ? (uint32_t)HAL_I2C_GetState(hi2c) : 0U;
}

/* 检查 I2C 句柄是否有效 */
static uint8_t AT24CS32_IsI2cReady(I2C_HandleTypeDef *hi2c)
{
    return (hi2c != NULL) ? 1U : 0U;
}

/*
 * 判断当前 I2C 句柄是否就是手柄认证所使用的 I2C2。
 * 当前现场问题主要集中在 I2C2，因此恢复逻辑会优先对 I2C2 做定向处理。
 */
static uint8_t AT24CS32_IsI2C2Handle(I2C_HandleTypeDef *hi2c)
{
    return (hi2c == &hi2c2) ? 1U : 0U;
}

/*
 * 在真正访问 EEPROM 前，先主动确认器件处于 ready 状态。
 * 这样可以把“器件未应答 / 总线未空闲”和“正式读写失败”区分开。
 * 现场分析时可以借助 HSDBG 报文更快判断问题发生在访问前还是访问中。
 */
static HAL_StatusTypeDef AT24CS32_WaitDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c,
                                 dev_addr,
                                 AT24CS32_READY_TRIALS,
                                 AT24CS32_I2C_TIMEOUT_MS);
}

/*
 * 判断当前失败是否值得执行一次 I2C 软恢复。
 * 这里重点关注 HAL_BUSY、HAL_TIMEOUT 以及 HAL_I2C_ERROR_TIMEOUT。
 * 因为这些情况都与现场看到的 BUSY Flag 超时高度一致。
 */
static uint8_t AT24CS32_ShouldRecoverI2c(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef hal_status)
{
    uint32_t hal_error;

    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return 0U;
    }

    hal_error = HAL_I2C_GetError(hi2c);

    if ((hal_status == HAL_BUSY) || (hal_status == HAL_TIMEOUT)) {
        return 1U;
    }

    if ((hal_error & HAL_I2C_ERROR_TIMEOUT) != 0U) {
        return 1U;
    }

    return 0U;
}

/*
 * 对 I2C 总线执行一次最小侵入的软恢复。
 * 当前优先采用 HAL DeInit + Init 的方式，把外设状态机从 BUSY/TIMEOUT 状态中拉回来。
 * 这样改动面较小，也更适合先验证是不是“外设状态卡死”导致的首包读取失败。
 */
static void AT24CS32_RecoverI2cBus(I2C_HandleTypeDef *hi2c)
{
    if (AT24CS32_IsI2cReady(hi2c) == 0U) {
        return;
    }

    if (AT24CS32_IsI2C2Handle(hi2c) != 0U) {
        HAL_I2C_DeInit(&hi2c2);
        MX_I2C2_Init();
    } else if (hi2c == &hi2c3) {
        HAL_I2C_DeInit(&hi2c3);
        MX_I2C3_Init();
    } else {
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);
    }

    HAL_Delay(AT24CS32_RECOVER_DELAY_MS);
}

/* 计算页起始字节地址 */
static uint16_t AT24CS32_PageToAddr(uint16_t page_index)
{
    return (uint16_t)(page_index * AT24CS32_PAGE_SIZE);
}

/* 封装 16 位字地址读接口 */
static uint8_t AT24CS32_MemRead(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    /*
     * 每次发起 EEPROM 读取前，先记录“调用前的 I2C 状态”。
     * 如果后面现场再次出现 BUSY/TIMEOUT，就能直接从调试报文里看出：
     * 是访问过程中卡住，还是发起访问之前总线就已经不干净了。
     */
    s_at24cs32_last_debug.op_type = AT24CS32_DEBUG_OP_READ;
    s_at24cs32_last_debug.dev_addr = dev_addr;
    s_at24cs32_last_debug.mem_addr = mem_addr;
    s_at24cs32_last_debug.data_len = len;
    s_at24cs32_last_debug.hal_status = 0xFFU;
    s_at24cs32_last_debug.hal_error = 0U;
    s_at24cs32_last_debug.i2c_state_before = (hi2c != NULL) ? (uint32_t)HAL_I2C_GetState(hi2c) : 0U;
    s_at24cs32_last_debug.i2c_state_after = 0U;

    /*
     * 第一步先做设备 ready 探测，而不是直接调用 Mem_Read。
     * 这样如果器件尚未 ready，或者总线在这里就已经 Busy，
     * 可以先尝试做一次恢复，避免把真正的读操作浪费掉。
     */
    hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
    if (hal_status != HAL_OK) {
        if (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U) {
            AT24CS32_RecoverI2cBus(hi2c);
            hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
        }

        AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status);
        if (hal_status != HAL_OK) {
            return 0U;
        }
    }

    /*
     * 设备 ready 后再执行真正的 EEPROM 读取。
     * 如果这里仍然遇到 BUSY/TIMEOUT，再做一次软恢复和单次重试。
     * 这样既能提升现场容错，又不会因为无限重试把问题掩盖掉。
     */
    hal_status = HAL_I2C_Mem_Read(hi2c,
                                  dev_addr,
                                  mem_addr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  buf,
                                  len,
                                  AT24CS32_I2C_TIMEOUT_MS);

    if ((hal_status != HAL_OK) && (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U)) {
        AT24CS32_RecoverI2cBus(hi2c);

        if (AT24CS32_WaitDeviceReady(hi2c, dev_addr) == HAL_OK) {
            hal_status = HAL_I2C_Mem_Read(hi2c,
                                          dev_addr,
                                          mem_addr,
                                          I2C_MEMADD_SIZE_16BIT,
                                          buf,
                                          len,
                                          AT24CS32_I2C_TIMEOUT_MS);
        } else {
            hal_status = HAL_BUSY;
        }
    }

    /* 无论最终成功还是失败，都把本次访问的真实结果完整回填到调试结构体里 */
    AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status);

    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* 封装 16 位字地址写接口 */
static uint8_t AT24CS32_MemWrite(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    /*
     * 写操作沿用与读操作一致的调试记录策略。
     * 这样后续如果现场转为排查“写入失败”问题，也能直接复用同一套报文分析思路。
     */
    s_at24cs32_last_debug.op_type = AT24CS32_DEBUG_OP_WRITE;
    s_at24cs32_last_debug.dev_addr = dev_addr;
    s_at24cs32_last_debug.mem_addr = mem_addr;
    s_at24cs32_last_debug.data_len = len;
    s_at24cs32_last_debug.hal_status = 0xFFU;
    s_at24cs32_last_debug.hal_error = 0U;
    s_at24cs32_last_debug.i2c_state_before = (hi2c != NULL) ? (uint32_t)HAL_I2C_GetState(hi2c) : 0U;
    s_at24cs32_last_debug.i2c_state_after = 0U;

    /*
     * 先确认器件可应答，再做真正的写入。
     * 对 EEPROM 来说，这一步也能避免“上一轮写周期尚未结束时又立即写入”带来的误判。
     */
    hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
    if (hal_status != HAL_OK) {
        if (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U) {
            AT24CS32_RecoverI2cBus(hi2c);
            hal_status = AT24CS32_WaitDeviceReady(hi2c, dev_addr);
        }

        AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);
        if (hal_status != HAL_OK) {
            return 0U;
        }
    }

    hal_status = HAL_I2C_Mem_Write(hi2c,
                                   dev_addr,
                                   mem_addr,
                                   I2C_MEMADD_SIZE_16BIT,
                                   buf,
                                   len,
                                   AT24CS32_I2C_TIMEOUT_MS);

    if ((hal_status != HAL_OK) && (AT24CS32_ShouldRecoverI2c(hi2c, hal_status) != 0U)) {
        AT24CS32_RecoverI2cBus(hi2c);

        if (AT24CS32_WaitDeviceReady(hi2c, dev_addr) == HAL_OK) {
            hal_status = HAL_I2C_Mem_Write(hi2c,
                                           dev_addr,
                                           mem_addr,
                                           I2C_MEMADD_SIZE_16BIT,
                                           buf,
                                           len,
                                           AT24CS32_I2C_TIMEOUT_MS);
        } else {
            hal_status = HAL_BUSY;
        }
    }

    /* 统一回填本次写操作的底层状态，便于现场直接对照 HSDBG 分析 */
    AT24CS32_UpdateDebugInfo(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);

    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* 通用序列号读取实现，按指定 I2C 句柄执行 */
static uint8_t AT24CS32_ReadSerialNumber_ByI2C(I2C_HandleTypeDef *hi2c, uint8_t *sn_buf)
{
    if ((sn_buf == NULL) || (AT24CS32_IsI2cReady(hi2c) == 0U)) {
        return 1U;
    }

    /* 序列号区固定地址为 0x0800，长度为 16 字节 */
    if (AT24CS32_MemRead(hi2c,
                         AT24CS32_SN_WRITE_ADDR,
                         0x0800U,
                         sn_buf,
                         AT24CS32_SN_SIZE) == 0U) {
        return 2U;
    }

    return 0U;
}

/* 通用写实现，按指定 I2C 句柄执行 */
static uint8_t AT24CS32_WriteBytes_ByI2C(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t len)
{
    if ((AT24CS32_IsI2cReady(hi2c) == 0U) || (data == NULL) ||
        (addr >= AT24CS32_TOTAL_SIZE) || (len == 0U)) {
        return 0U;
    }

    if ((uint32_t)addr + len > AT24CS32_TOTAL_SIZE) {
        return 0U;
    }

    while (len != 0U) {
        /* 计算当前页剩余空间，保证单次写不跨页 */
        uint16_t page_left = (uint16_t)(AT24CS32_PAGE_SIZE - (addr % AT24CS32_PAGE_SIZE));
        /* 本次实际写入长度 */
        uint16_t write_len = (len > page_left) ? page_left : len;

        /* 执行一次分页写 */
        if (AT24CS32_MemWrite(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, data, write_len) == 0U) {
            return 0U;
        }

        /* 等待 EEPROM 写周期完成 */
        HAL_Delay(5);

        /* 更新地址、指针和剩余长度 */
        addr += write_len;
        data += write_len;
        len -= write_len;
    }
    return 1U;
}

/* 通用读实现，按指定 I2C 句柄执行 */
static uint8_t AT24CS32_ReadBytes_ByI2C(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t len)
{
    if ((AT24CS32_IsI2cReady(hi2c) == 0U) || (data == NULL) ||
        (addr >= AT24CS32_TOTAL_SIZE) || (len == 0U)) {
        return 0U;
    }

    if ((uint32_t)addr + len > AT24CS32_TOTAL_SIZE) {
        return 0U;
    }

    while (len != 0U) {
        /* 每次读取固定分块，提升稳定性 */
        uint16_t read_len = (len > AT24CS32_READ_CHUNK_SIZE) ? AT24CS32_READ_CHUNK_SIZE : len;

        /* 执行一次分块读 */
        if (AT24CS32_MemRead(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, data, read_len) == 0U) {
            return 0U;
        }

        /* 更新地址、指针和剩余长度 */
        addr += read_len;
        data += read_len;
        len -= read_len;
    }

    return 1U;
}

/* 计算页和校验值：对前 30 字节做 16 位累加 */
uint16_t AT24CS32_PageChecksum(const uint8_t *page_buf)
{
    uint16_t sum = 0U;
    uint8_t i;

    if (page_buf == NULL) {
        return 0U;
    }

    for (i = 0U; i < AT24CS32_PAGE_DATA_SIZE; i++) {
        sum = (uint16_t)(sum + page_buf[i]);
    }

    return sum;
}

/* 校验页尾页和：page[30] 为高字节，page[31] 为低字节 */
uint8_t AT24CS32_VerifyPageChecksum(const uint8_t *page_buf)
{
    uint16_t calc_sum;
    uint16_t stored_sum;

    if (page_buf == NULL) {
        return 0U;
    }

    calc_sum = AT24CS32_PageChecksum(page_buf);
    stored_sum = (uint16_t)(((uint16_t)page_buf[AT24CS32_PAGE_SIZE - 2U] << 8) |
                            page_buf[AT24CS32_PAGE_SIZE - 1U]);

    return (calc_sum == stored_sum) ? 1U : 0U;
}

/* 通用按页读取：读取整页后立即做页和校验 */
static uint8_t AT24CS32_ReadPage_ByI2C(I2C_HandleTypeDef *hi2c, uint16_t page_index, uint8_t *page_buf)
{
    uint16_t addr;

    if ((AT24CS32_IsI2cReady(hi2c) == 0U) || (page_buf == NULL) ||
        (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    addr = AT24CS32_PageToAddr(page_index);
    if (AT24CS32_MemRead(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, page_buf, AT24CS32_PAGE_SIZE) == 0U) {
        return 0U;
    }

    if (AT24CS32_VerifyPageChecksum(page_buf) == 0U) {
        return 0U;
    }

    return 1U;
}

/* 通用按页写入：写入前自动刷新页和校验 */
static uint8_t AT24CS32_WritePage_ByI2C(I2C_HandleTypeDef *hi2c, uint16_t page_index, const uint8_t *page_buf)
{
    uint16_t addr;
    uint16_t sum;
    uint8_t temp_page[AT24CS32_PAGE_SIZE];

    if ((AT24CS32_IsI2cReady(hi2c) == 0U) || (page_buf == NULL) ||
        (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    memcpy(temp_page, page_buf, AT24CS32_PAGE_SIZE);
    sum = AT24CS32_PageChecksum(temp_page);
    temp_page[AT24CS32_PAGE_SIZE - 2U] = (uint8_t)(sum >> 8);
    temp_page[AT24CS32_PAGE_SIZE - 1U] = (uint8_t)(sum & 0xFFU);

    addr = AT24CS32_PageToAddr(page_index);
    if (AT24CS32_MemWrite(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, temp_page, AT24CS32_PAGE_SIZE) == 0U) {
        return 0U;
    }

    HAL_Delay(5);
    return 1U;
}

/* I2C2 显式接口：读取序列号 */
uint8_t AT24CS32_ReadSerialNumber_I2C2(uint8_t *sn_buf)
{
    return AT24CS32_ReadSerialNumber_ByI2C(&hi2c2, sn_buf);
}

/* I2C3 显式接口：读取序列号 */
uint8_t AT24CS32_ReadSerialNumber_I2C3(uint8_t *sn_buf)
{
    return AT24CS32_ReadSerialNumber_ByI2C(&hi2c3, sn_buf);
}

/* I2C2 显式接口：写数据 */
uint8_t AT24CS32_WriteBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len)
{
    return AT24CS32_WriteBytes_ByI2C(&hi2c2, addr, data, len);
}

/* I2C3 显式接口：写数据 */
uint8_t AT24CS32_WriteBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len)
{
    return AT24CS32_WriteBytes_ByI2C(&hi2c3, addr, data, len);
}

/* I2C2 显式接口：读数据 */
uint8_t AT24CS32_ReadBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len)
{
    return AT24CS32_ReadBytes_ByI2C(&hi2c2, addr, data, len);
}

/* I2C3 显式接口：读数据 */
uint8_t AT24CS32_ReadBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len)
{
    return AT24CS32_ReadBytes_ByI2C(&hi2c3, addr, data, len);
}

/* I2C2 显式接口：按页读取 */
uint8_t AT24CS32_ReadPage_I2C2(uint16_t page_index, uint8_t *page_buf)
{
    return AT24CS32_ReadPage_ByI2C(&hi2c2, page_index, page_buf);
}

/* I2C3 显式接口：按页读取 */
uint8_t AT24CS32_ReadPage_I2C3(uint16_t page_index, uint8_t *page_buf)
{
    return AT24CS32_ReadPage_ByI2C(&hi2c3, page_index, page_buf);
}

/* I2C2 显式接口：按页写入 */
uint8_t AT24CS32_WritePage_I2C2(uint16_t page_index, const uint8_t *page_buf)
{
    return AT24CS32_WritePage_ByI2C(&hi2c2, page_index, page_buf);
}

/* I2C3 显式接口：按页写入 */
uint8_t AT24CS32_WritePage_I2C3(uint16_t page_index, const uint8_t *page_buf)
{
    return AT24CS32_WritePage_ByI2C(&hi2c3, page_index, page_buf);
}

/*
 * 清空最近一次调试信息。
 * 上层在准备开始一次“关键认证读取”之前先调用这个函数，
 * 后续若发生失败，就能确认读到的内容一定对应本轮访问。
 */
void AT24CS32_ClearLastDebugInfo(void)
{
    memset(&s_at24cs32_last_debug, 0, sizeof(s_at24cs32_last_debug));
}

/*
 * 读取最近一次 EEPROM I2C 访问调试信息。
 * 使用值拷贝，避免上层直接改动底层保存的原始调试结果。
 */
void AT24CS32_GetLastDebugInfo(AT24CS32_DebugInfo *info)
{
    if (info == NULL) {
        return;
    }

    memcpy(info, &s_at24cs32_last_debug, sizeof(*info));
}







