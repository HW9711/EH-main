#include "at24cs32_crc_verify.h"
#include <string.h>

/* Page1 整页缓冲（32B，用于读取并校验页和） */
static uint8_t s_page1_buf[AT24CS32_PAGE_SIZE];
/* Page2~Page8 数据缓冲（224B，包含每页最后2字节页和） */
static uint8_t s_auth_data[AT24CS32_AUTH_DATA_LENGTH];
/* CRC输入缓冲：SN(16B) + Page2~8(224B) */
static uint8_t s_crc_input[AT24CS32_AUTH_INPUT_LENGTH];

/* 按总线做原始整页读取（不执行页和校验，用于区分读失败与页和失败） */
static uint8_t At24_ReadRawPage(uint8_t use_i2c3, uint16_t page_index, uint8_t *page_buf)
{
    uint16_t addr;

    /* 输出缓存为空或页号越界时拒绝访问，避免 EEPROM 地址回卷和内存写坏。 */
    if ((page_buf == NULL) || (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    addr = (uint16_t)(page_index * AT24CS32_PAGE_SIZE);
    return use_i2c3 ?
        AT24CS32_ReadBytes_I2C3(addr, page_buf, AT24CS32_PAGE_SIZE) :
        AT24CS32_ReadBytes_I2C2(addr, page_buf, AT24CS32_PAGE_SIZE);
}

/* 通用 CRC16 计算：初值和多项式由宏配置 */
static uint16_t At24_Crc16(const uint8_t *data, uint32_t len, uint16_t init, uint16_t poly)
{
    uint16_t crc = init;
    uint32_t i;
    uint8_t j;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0U; j < 8U; j++) {
            /* CRC 最高位为 1 时移位后异或多项式，否则只左移，保持约定的非反射算法。 */
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/*
 * 函数功能：每次只读取一个认证块，完整校验通过后发布本通道的业务页快照。
 * 输入参数：use_i2c3为0选I2C2、1选I2C3；context由调用方独占，新一轮将step归零。
 * 返回参数：PENDING表示下周期继续；OK表示全部通过；其它值保持原认证错误含义。
 */
AT24CS32_CRC_Status AT24CS32_VerifyCrcStep(uint8_t use_i2c3, AT24CS32_CRC_StepContext *context)
{
    static const uint16_t initial[4] = {AT24CS32_CRC16_INIT_1, AT24CS32_CRC16_INIT_2, AT24CS32_CRC16_INIT_3, AT24CS32_CRC16_INIT_4}; /* 四组初值沿用原认证协议。 */
    static const uint16_t polynomial[4] = {AT24CS32_CRC16_POLY_1, AT24CS32_CRC16_POLY_2, AT24CS32_CRC16_POLY_3, AT24CS32_CRC16_POLY_4}; /* 四组多项式顺序不能改变。 */
    uint8_t page1[AT24CS32_PAGE_SIZE]; /* Page1只在本次调用中读取，其认证值另存到通道上下文。 */
    uint8_t *block; /* 当前参与累计CRC的SN或业务页，不指向另一通道。 */
    uint16_t length; /* 本轮CRC输入长度，SN为16字节、业务页为32字节。 */
    uint8_t index; /* 遍历原协议的四组CRC。 */

    if ((context == NULL) || (use_i2c3 > 1U) || (context->step > AT24CS32_AUTH_PAGE_COUNT + 1U)) {
        return AT24CS32_CRC_STATUS_BAD_PARAM; /* 非法或已完成上下文必须由调用方重新初始化，禁止越界读页。 */
    }
    if (context->step == 0U) {
        context->verified = 0U; /* 先撤销上轮快照资格，任何读取失败都不能沿用旧认证结果。 */
        memset(&context->result, 0, sizeof(context->result)); /* 新一轮SN和结果从空状态开始。 */
        if (At24_ReadRawPage(use_i2c3, AT24CS32_AUTH_PAGE1_INDEX, page1) == 0U) {
            return AT24CS32_CRC_STATUS_PAGE1_READ_FAILED; /* Page1读失败沿用原错误码及上层重试。 */
        }
        if (AT24CS32_VerifyPageChecksum(page1) == 0U) {
            return AT24CS32_CRC_STATUS_PAGE1_CHECKSUM_FAILED; /* 页和失败不能进入后续认证。 */
        }
        memcpy(context->result.stored_auth, page1, AT24CS32_AUTH_RESULT_SIZE); /* 保存本通道Page1认证值。 */
        memcpy(context->crc, initial, sizeof(initial)); /* 四组CRC从各自协议初值开始累计。 */
        context->step = 1U; /* 下轮再读取SN，释放公共业务锁给泵通信。 */
        return AT24CS32_CRC_STATUS_PENDING; /* 分步等待不消耗认证失败次数。 */
    }
    if (context->step == 1U) {
        if ((use_i2c3 ? AT24CS32_ReadSerialNumber_I2C3(context->result.sn) :
                       AT24CS32_ReadSerialNumber_I2C2(context->result.sn)) != 0U) {
            return AT24CS32_CRC_STATUS_SN_READ_FAILED; /* SN接口以0表示成功，失败时不能累计CRC。 */
        }
        block = context->result.sn; /* 原协议先将SN作为认证输入。 */
        length = AT24CS32_SN_SIZE; /* 仅处理真实16字节SN。 */
    } else {
        block = &context->pages[(context->step - 2U) * AT24CS32_PAGE_SIZE]; /* 第2步对应Page2，快照偏移从0开始。 */
        if ((use_i2c3 ? AT24CS32_ReadPage_I2C3(context->step - 1U, block) :
                       AT24CS32_ReadPage_I2C2(context->step - 1U, block)) == 0U) {
            return AT24CS32_CRC_STATUS_DATA_READ_FAILED; /* 任一页读取或页和失败立即拒绝整轮快照。 */
        }
        length = AT24CS32_PAGE_SIZE; /* 页尾两字节也参与CRC，与旧认证输入完全一致。 */
    }
    for (index = 0U; index < 4U; ++index) {
        context->crc[index] = At24_Crc16(block, length, context->crc[index], polynomial[index]); /* 接续上一块CRC，不在页面边界重新使用初值。 */
    }
    ++context->step; /* 当前块完整读取并累计后才推进，下一周期不会跳页。 */
    if (context->step <= AT24CS32_AUTH_PAGE_COUNT + 1U) {
        return AT24CS32_CRC_STATUS_PENDING; /* 尚未读完Page8时保持认证中，不允许业务上线。 */
    }
    for (index = 0U; index < 4U; ++index) {
        context->result.calculated_auth[index * 2U] = (uint8_t)(context->crc[index] >> 8); /* 每组CRC高字节在前。 */
        context->result.calculated_auth[index * 2U + 1U] = (uint8_t)context->crc[index]; /* 低字节紧随，保持原八字节认证布局。 */
    }
    if (memcmp(context->result.stored_auth, context->result.calculated_auth, AT24CS32_AUTH_RESULT_SIZE) != 0) {
        return AT24CS32_CRC_STATUS_CRC_MISMATCH; /* 四组中任一结果不符都拒绝，不能发布半可信参数。 */
    }
    context->verified = 1U; /* 所有认证条件同时通过后，才允许本轮业务装载复用快照。 */
    return AT24CS32_CRC_STATUS_OK; /* 上层下一周期进入参数装载，不继续占用本轮锁。 */
}

/* 按总线读取 Page2~Page8（共7页，224字节），每页读取时都会做页和校验 */
static uint8_t At24_ReadAuthPages(uint8_t use_i2c3, uint8_t *out_buf)
{
    uint8_t page_buf[AT24CS32_PAGE_SIZE];
    uint16_t page_idx;
    uint16_t offset = 0U;

    /* 调用方未提供认证数据缓存时拒绝读取，避免连续七页数据写入空地址。 */
    if (out_buf == NULL) {
        return 0U;
    }

    for (page_idx = AT24CS32_AUTH_PAGE_START_INDEX;
         page_idx < (AT24CS32_AUTH_PAGE_START_INDEX + AT24CS32_AUTH_PAGE_COUNT);
         page_idx++) {
        uint8_t ok = use_i2c3 ?
            AT24CS32_ReadPage_I2C3(page_idx, page_buf) :
            AT24CS32_ReadPage_I2C2(page_idx, page_buf);

        /* 任一认证页读取或页和校验失败都立即终止，避免用不完整数据计算认证 CRC。 */
        if (ok == 0U) {
            return 0U;
        }

        memcpy(&out_buf[offset], page_buf, AT24CS32_PAGE_SIZE);
        offset = (uint16_t)(offset + AT24CS32_PAGE_SIZE);
    }

    return 1U;
}

/* 按总线执行认证校验：use_i2c3=0 使用 I2C2，use_i2c3=1 使用 I2C3 */
static AT24CS32_CRC_Status At24_VerifyBus(uint8_t use_i2c3, AT24CS32_CRC_Result *result)
{
    uint16_t crc1;
    uint16_t crc2;
    uint16_t crc3;
    uint16_t crc4;

    /* 结果结构为空时不能保存序列号和认证值，直接返回参数错误。 */
    if (result == NULL) {
        return AT24CS32_CRC_STATUS_BAD_PARAM;
    }

    memset(result, 0, sizeof(*result));

    /*
     * 先读取 Page1 并验证页和（Page1[30:31]只用于页和校验，不参与认证结果比较）：
     * - 先做原始读取，失败则返回 PAGE1_READ_FAILED
     * - 读取成功后单独校验页和，失败则返回 PAGE1_CHECKSUM_FAILED
     */
    /* Page1 原始读取失败时单独上报读失败，不能误判成页和或认证值错误。 */
    if (At24_ReadRawPage(use_i2c3, AT24CS32_AUTH_PAGE1_INDEX, s_page1_buf) == 0U) {
        return AT24CS32_CRC_STATUS_PAGE1_READ_FAILED;
    }

    /* Page1 页和不符说明记录已损坏，后续认证字段不再参与比较。 */
    if (AT24CS32_VerifyPageChecksum(s_page1_buf) == 0U) {
        return AT24CS32_CRC_STATUS_PAGE1_CHECKSUM_FAILED;
    }

    memcpy(result->stored_auth, s_page1_buf, AT24CS32_AUTH_RESULT_SIZE);

    /* 读取 SN */
    /* B 通道认证选择 I2C3 序列号区，A 通道则走下方 I2C2 分支。 */
    if (use_i2c3) {
        /* I2C3 序列号读取接口非零表示失败，认证输入无法继续构造。 */
        if (AT24CS32_ReadSerialNumber_I2C3(result->sn) != 0U) {
            return AT24CS32_CRC_STATUS_SN_READ_FAILED;
        }
    } else {
        /* I2C2 序列号读取接口非零表示失败，A 通道认证立即退出。 */
        if (AT24CS32_ReadSerialNumber_I2C2(result->sn) != 0U) {
            return AT24CS32_CRC_STATUS_SN_READ_FAILED;
        }
    }

    /* 读取 Page2~Page8（0x0020~0x00FF）并按页校验 */
    /* Page2~8 任一页失败会造成认证输入不完整，因此不能继续计算 CRC。 */
    if (At24_ReadAuthPages(use_i2c3, s_auth_data) == 0U) {
        return AT24CS32_CRC_STATUS_DATA_READ_FAILED;
    }

    /* 认证输入为 SN + Page2~8(含每页末尾2字节页和) */
    memcpy(s_crc_input, result->sn, AT24CS32_SN_SIZE);
    memcpy(&s_crc_input[AT24CS32_SN_SIZE], s_auth_data, AT24CS32_AUTH_DATA_LENGTH);

    /* 同一输入做4组CRC16，拼接为8字节（大端） */
    crc1 = At24_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_1, AT24CS32_CRC16_POLY_1);
    crc2 = At24_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_2, AT24CS32_CRC16_POLY_2);
    crc3 = At24_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_3, AT24CS32_CRC16_POLY_3);
    crc4 = At24_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_4, AT24CS32_CRC16_POLY_4);

    result->calculated_auth[0] = (uint8_t)(crc1 >> 8);
    result->calculated_auth[1] = (uint8_t)(crc1 & 0xFFU);
    result->calculated_auth[2] = (uint8_t)(crc2 >> 8);
    result->calculated_auth[3] = (uint8_t)(crc2 & 0xFFU);
    result->calculated_auth[4] = (uint8_t)(crc3 >> 8);
    result->calculated_auth[5] = (uint8_t)(crc3 & 0xFFU);
    result->calculated_auth[6] = (uint8_t)(crc4 >> 8);
    result->calculated_auth[7] = (uint8_t)(crc4 & 0xFFU);

    /* 与 Page1 前8字节比对 */
    /* 计算值与 Page1 保存值任一字节不同都视为认证失败，禁止把该手柄当成有效设备。 */
    if (memcmp(result->stored_auth, result->calculated_auth, AT24CS32_AUTH_RESULT_SIZE) != 0) {
        return AT24CS32_CRC_STATUS_CRC_MISMATCH;
    }

    return AT24CS32_CRC_STATUS_OK;
}

/* I2C2 显式接口：按固定布局执行认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C2(AT24CS32_CRC_Result *result)
{
    return At24_VerifyBus(0U, result);
}

/* I2C3 显式接口：按固定布局执行认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C3(AT24CS32_CRC_Result *result)
{
    return At24_VerifyBus(1U, result);
}

/* I2C2 兼容接口：默认布局认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrcDefault_I2C2(AT24CS32_CRC_Result *result)
{
    return AT24CS32_VerifyCrc_I2C2(result);
}

/* I2C3 兼容接口：默认布局认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrcDefault_I2C3(AT24CS32_CRC_Result *result)
{
    return AT24CS32_VerifyCrc_I2C3(result);
}
