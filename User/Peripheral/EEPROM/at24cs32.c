#include "at24cs32.h"
#include <string.h>

/* 写操作及写前应答探测仍使用100ms超时参数，不改变原有写入重试。 */
#define AT24CS32_I2C_TIMEOUT_MS        100U
#define AT24CS32_READ_TIMEOUT_MS     10U /* 应答探测和读取共用10ms超时时间；HAL内部还可能等待总线忙状态最多25ms，因此整个函数可能超过10ms。 */
/* 连续读取一次最多传输的字节数；64字节读完再读下一块，不是EEPROM的物理页大小。 */
#define AT24CS32_READ_CHUNK_SIZE       64U
/* 读写前最多检查EEPROM应答的次数；调大可能延长未连接手柄的等待时间。 */
#define AT24CS32_READY_TRIALS          3U
/* I2C忙或超时后，重新初始化总线并等待的毫秒数；等待结束后才重试访问。 */
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
static void At24_UpdateDebug(I2C_HandleTypeDef *hi2c,
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
static uint8_t At24_IsReady(I2C_HandleTypeDef *hi2c)
{
    return (hi2c != NULL) ? 1U : 0U;
}

/*
 * 函数功能：检查传入的是否为I2C2，以便恢复时选择对应的初始化函数。
 * 输入参数：hi2c为待检查的I2C接口。
 * 返回参数：1表示I2C2，0表示其它接口或空指针。
 */
static uint8_t At24_IsI2c2(I2C_HandleTypeDef *hi2c)
{
    return (hi2c == &hi2c2) ? 1U : 0U;
}

/*
 * 在真正访问 EEPROM 前，先主动确认器件处于 ready 状态。
 * 这样可以把“器件未应答 / 总线未空闲”和“正式读写失败”区分开。
 * 现场分析时可以借助 HSDBG 报文更快判断问题发生在访问前还是访问中。
 */
static HAL_StatusTypeDef At24_WaitReady(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    /* I2C 句柄为空时不能访问 HAL 状态机，直接返回错误给上层恢复链。 */
    if (At24_IsReady(hi2c) == 0U) {
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
static uint8_t At24_NeedsRecovery(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef hal_status)
{
    uint32_t hal_error;

    /* 无有效 I2C 句柄时不存在可恢复的硬件状态，禁止读取 HAL 错误码。 */
    if (At24_IsReady(hi2c) == 0U) {
        return 0U;
    }

    hal_error = HAL_I2C_GetError(hi2c);

    /* HAL 明确报告总线忙或超时时允许软恢复，给本次 EEPROM 访问一次重试机会。 */
    if ((hal_status == HAL_BUSY) || (hal_status == HAL_TIMEOUT)) {
        return 1U;
    }

    /* HAL 状态未直接返回超时但错误位已置位时也需要恢复外设状态机。 */
    if ((hal_error & HAL_I2C_ERROR_TIMEOUT) != 0U) {
        return 1U;
    }

    return 0U;
}

/*
 * 函数功能：关闭并重新初始化出错的I2C接口，等待后让上层重试EEPROM访问。
 * 输入参数：hi2c为要恢复的I2C接口；空指针不执行操作。
 * 返回参数：无；重新初始化不代表下一次读写一定成功。
 */
static void At24_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    /* 句柄为空时没有可恢复的总线，静默返回以保持其它 I2C 通道不变。 */
    if (At24_IsReady(hi2c) == 0U) {
        return;
    }

    /* 恢复I2C2的引脚、时钟和速度；屏幕A/B归属由手柄接口交换配置决定。 */
    if (At24_IsI2c2(hi2c) != 0U) {
        HAL_I2C_DeInit(&hi2c2);
        MX_I2C2_Init();
    /* 恢复I2C3，不能错误地使用I2C2的引脚配置。 */
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
static uint16_t At24_PageAddr(uint16_t page_index)
{
    return (uint16_t)(page_index * AT24CS32_PAGE_SIZE);
}

/*
 * 函数功能：读取EEPROM时缩短应答探测和读取等待；失败后恢复总线并返回，由调用方以后重试。
 * 输入参数：hi2c为总线，dev_addr为器件地址，mem_addr为16位字地址，buf/len为接收缓存及长度。
 * 返回参数：1表示本次完整读取成功；0表示失败，不在同一调用中重复阻塞读取。
 */
static uint8_t At24_MemRead(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_status;
    uint32_t started = HAL_GetTick(); /* 记录本次开始时间，后面的应答探测和数据读取共同使用这10ms。 */
    uint32_t elapsed; /* 无符号差值支持毫秒计数回绕。 */

    if ((hi2c == NULL) || (buf == NULL) || (len == 0U)) {
        return 0U; /* 参数无效时不访问硬件，避免访问空指针。 */
    }

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

    hal_status = HAL_I2C_IsDeviceReady(hi2c, dev_addr, 1U, AT24CS32_READ_TIMEOUT_MS); /* 只读每轮只探测一次，失败由扫描状态机下轮重试；写前探测仍保持原三次。 */
    if (hal_status != HAL_OK) {
        At24_UpdateDebug(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status); /* 恢复前保存本次真实故障，避免HAL初始化清除错误证据。 */
        if (At24_NeedsRecovery(hi2c, hal_status) != 0U) {
            At24_RecoverBus(hi2c); /* 本轮只恢复总线，不立即再次探测或读页，给泵通信让出执行机会。 */
        }
        return 0U; /* 本次失败仍交给原有认证限次重试和报警规则，不将恢复动作冒充读成功。 */
    }

    elapsed = (uint32_t)(HAL_GetTick() - started); /* 检查应答阶段已经占用的墙钟时间。 */
    if (elapsed >= AT24CS32_READ_TIMEOUT_MS) {
        At24_UpdateDebug(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, HAL_TIMEOUT); /* 应答探测已用完允许的等待时间，本次直接返回，不再读取数据。 */
        return 0U; /* 设备已经应答，不额外复位总线，等待上层下一轮重新读取。 */
    }
    hal_status = HAL_I2C_Mem_Read(hi2c,
                                  dev_addr,
                                  mem_addr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  buf,
                                  len,
                                  AT24CS32_READ_TIMEOUT_MS - elapsed); /* 将应答探测剩余的毫秒数传给数据读取，避免继续等待100ms。 */

    At24_UpdateDebug(hi2c, AT24CS32_DEBUG_OP_READ, dev_addr, mem_addr, len, hal_status); /* 完整记录读取结果；总线恢复不能覆盖本次错误证据。 */
    if ((hal_status != HAL_OK) && (At24_NeedsRecovery(hi2c, hal_status) != 0U)) {
        At24_RecoverBus(hi2c); /* 保留总线恢复能力，但本轮不再追加第二次读操作。 */
    }

    /* HAL 最终返回成功时向上层报告有效数据，其它状态统一报告失败。 */
    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* 封装 16 位字地址写接口 */
static uint8_t At24_MemWrite(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr, uint8_t *buf, uint16_t len)
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
    hal_status = At24_WaitReady(hi2c, dev_addr);
    /* 器件未结束上一轮写周期时不能直接写入，先进入一次受限恢复流程。 */
    if (hal_status != HAL_OK) {
        /* 只有 BUSY/TIMEOUT 类状态才重建 I2C，普通未应答不反复复位总线。 */
        if (At24_NeedsRecovery(hi2c, hal_status) != 0U) {
            At24_RecoverBus(hi2c);
            hal_status = At24_WaitReady(hi2c, dev_addr);
        }

        At24_UpdateDebug(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);
        /* 恢复后仍未就绪时停止写入，避免把页数据写成不完整记录。 */
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

    /* 正式写入遇到可恢复故障时只重试一次，避免 EEPROM 任务长时间占用 I2C。 */
    if ((hal_status != HAL_OK) && (At24_NeedsRecovery(hi2c, hal_status) != 0U)) {
        At24_RecoverBus(hi2c);

        /* 总线恢复且器件应答后才允许再次写页，防止在 BUSY 状态重复下发。 */
        if (At24_WaitReady(hi2c, dev_addr) == HAL_OK) {
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
    At24_UpdateDebug(hi2c, AT24CS32_DEBUG_OP_WRITE, dev_addr, mem_addr, len, hal_status);

    /* HAL 最终成功才向上层确认写入已发出，其它状态统一报告失败。 */
    if (hal_status == HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* 通用序列号读取实现，按指定 I2C 句柄执行 */
static uint8_t At24_ReadSn(I2C_HandleTypeDef *hi2c, uint8_t *sn_buf)
{
    /* 序列号缓存或 I2C 句柄无效时拒绝读取，避免认证输入写入空地址。 */
    if ((sn_buf == NULL) || (At24_IsReady(hi2c) == 0U)) {
        return 1U;
    }

    /* 序列号区固定地址为 0x0800，长度为 16 字节 */
    /* 固定序列号区读取失败时返回独立错误码，便于认证层区分参数错误。 */
    if (At24_MemRead(hi2c,
                         AT24CS32_SN_WRITE_ADDR,
                         0x0800U,
                         sn_buf,
                         AT24CS32_SN_SIZE) == 0U) {
        return 2U;
    }

    return 0U;
}

/* 通用写实现，按指定 I2C 句柄执行 */
static uint8_t At24_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t len)
{
    /* 句柄、缓存、地址或长度非法时拒绝写入，防止 EEPROM 越界和空指针访问。 */
    if ((At24_IsReady(hi2c) == 0U) || (data == NULL) ||
        (addr >= AT24CS32_TOTAL_SIZE) || (len == 0U)) {
        return 0U;
    }

    /* 起始地址有效但末地址越界时同样拒绝，防止跨过 AT24CS32 末尾。 */
    if ((uint32_t)addr + len > AT24CS32_TOTAL_SIZE) {
        return 0U;
    }

    while (len != 0U) {
        /* 计算当前页剩余空间，保证单次写不跨页 */
        uint16_t page_left = (uint16_t)(AT24CS32_PAGE_SIZE - (addr % AT24CS32_PAGE_SIZE));
        /* 本次实际写入长度 */
        uint16_t write_len = (len > page_left) ? page_left : len;

        /* 执行一次分页写 */
        /* 任一分页写失败都立即停止，避免后续页继续写入形成半份记录。 */
        if (At24_MemWrite(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, data, write_len) == 0U) {
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
static uint8_t At24_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t len)
{
    /* 句柄、缓存、地址或长度非法时拒绝读取，避免越界访问和无效输出。 */
    if ((At24_IsReady(hi2c) == 0U) || (data == NULL) ||
        (addr >= AT24CS32_TOTAL_SIZE) || (len == 0U)) {
        return 0U;
    }

    /* 读取末地址超过器件容量时拒绝操作，避免地址回卷读到错误数据。 */
    if ((uint32_t)addr + len > AT24CS32_TOTAL_SIZE) {
        return 0U;
    }

    while (len != 0U) {
        /* 每次读取固定分块，提升稳定性 */
        uint16_t read_len = (len > AT24CS32_READ_CHUNK_SIZE) ? AT24CS32_READ_CHUNK_SIZE : len;

        /* 执行一次分块读 */
        /* 任一分块读取失败都立即停止，避免上层使用前半段有效、后半段旧值的缓存。 */
        if (At24_MemRead(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, data, read_len) == 0U) {
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

    /* 页缓存为空时无法计算校验和，返回 0 并保持内存不访问。 */
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

    /* 页缓存为空时不能读取页尾校验值，直接判定校验失败。 */
    if (page_buf == NULL) {
        return 0U;
    }

    calc_sum = AT24CS32_PageChecksum(page_buf);
    stored_sum = (uint16_t)(((uint16_t)page_buf[AT24CS32_PAGE_SIZE - 2U] << 8) |
                            page_buf[AT24CS32_PAGE_SIZE - 1U]);

    return (calc_sum == stored_sum) ? 1U : 0U;
}

/* 通用按页读取：读取整页后立即做页和校验 */
static uint8_t At24_ReadPage(I2C_HandleTypeDef *hi2c, uint16_t page_index, uint8_t *page_buf)
{
    uint16_t addr;

    /* 句柄、页缓存或页号非法时拒绝整页读取，避免地址越界。 */
    if ((At24_IsReady(hi2c) == 0U) || (page_buf == NULL) ||
        (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    addr = At24_PageAddr(page_index);
    /* 整页读取失败时不再做校验，防止旧缓存被误认为本次有效数据。 */
    if (At24_MemRead(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, page_buf, AT24CS32_PAGE_SIZE) == 0U) {
        return 0U;
    }

    /* 页尾校验和不匹配说明 EEPROM 数据损坏，整页结果必须判失败。 */
    if (AT24CS32_VerifyPageChecksum(page_buf) == 0U) {
        return 0U;
    }

    return 1U;
}

/* 通用按页写入：写入前自动刷新页和校验 */
static uint8_t At24_WritePage(I2C_HandleTypeDef *hi2c, uint16_t page_index, const uint8_t *page_buf)
{
    uint16_t addr;
    uint16_t sum;
    uint8_t temp_page[AT24CS32_PAGE_SIZE];

    /* 句柄、页缓存或页号非法时拒绝整页写入，避免覆盖错误 EEPROM 区域。 */
    if ((At24_IsReady(hi2c) == 0U) || (page_buf == NULL) ||
        (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    memcpy(temp_page, page_buf, AT24CS32_PAGE_SIZE);
    sum = AT24CS32_PageChecksum(temp_page);
    temp_page[AT24CS32_PAGE_SIZE - 2U] = (uint8_t)(sum >> 8);
    temp_page[AT24CS32_PAGE_SIZE - 1U] = (uint8_t)(sum & 0xFFU);

    addr = At24_PageAddr(page_index);
    /* 带新页和的整页写入失败时向上层报告失败，不能假定 EEPROM 已落盘。 */
    if (At24_MemWrite(hi2c, AT24CS32_EEPROM_WRITE_ADDR, addr, temp_page, AT24CS32_PAGE_SIZE) == 0U) {
        return 0U;
    }

    HAL_Delay(5);
    return 1U;
}

/* I2C2 显式接口：读取序列号 */
uint8_t AT24CS32_ReadSerialNumber_I2C2(uint8_t *sn_buf)
{
    return At24_ReadSn(&hi2c2, sn_buf);
}

/* I2C3 显式接口：读取序列号 */
uint8_t AT24CS32_ReadSerialNumber_I2C3(uint8_t *sn_buf)
{
    return At24_ReadSn(&hi2c3, sn_buf);
}

/* I2C2 显式接口：写数据 */
uint8_t AT24CS32_WriteBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len)
{
    return At24_WriteBytes(&hi2c2, addr, data, len);
}

/* I2C3 显式接口：写数据 */
uint8_t AT24CS32_WriteBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len)
{
    return At24_WriteBytes(&hi2c3, addr, data, len);
}

/* I2C2 显式接口：读数据 */
uint8_t AT24CS32_ReadBytes_I2C2(uint16_t addr, uint8_t *data, uint16_t len)
{
    return At24_ReadBytes(&hi2c2, addr, data, len);
}

/* I2C3 显式接口：读数据 */
uint8_t AT24CS32_ReadBytes_I2C3(uint16_t addr, uint8_t *data, uint16_t len)
{
    return At24_ReadBytes(&hi2c3, addr, data, len);
}

/* I2C2 显式接口：按页读取 */
uint8_t AT24CS32_ReadPage_I2C2(uint16_t page_index, uint8_t *page_buf)
{
    return At24_ReadPage(&hi2c2, page_index, page_buf);
}

/* I2C3 显式接口：按页读取 */
uint8_t AT24CS32_ReadPage_I2C3(uint16_t page_index, uint8_t *page_buf)
{
    return At24_ReadPage(&hi2c3, page_index, page_buf);
}

/* I2C2 显式接口：按页写入 */
uint8_t AT24CS32_WritePage_I2C2(uint16_t page_index, const uint8_t *page_buf)
{
    return At24_WritePage(&hi2c2, page_index, page_buf);
}

/* I2C3 显式接口：按页写入 */
uint8_t AT24CS32_WritePage_I2C3(uint16_t page_index, const uint8_t *page_buf)
{
    return At24_WritePage(&hi2c3, page_index, page_buf);
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
    /* 调用方未提供输出结构时不复制调试状态，避免空指针写入。 */
    if (info == NULL) {
        return;
    }

    memcpy(info, &s_at24cs32_last_debug, sizeof(*info));
}







