#include "at24cs32_crc_verify.h"
#include <string.h>

/* Page1 整页缓冲（32B，用于读取并校验页和） */
static uint8_t s_page1_buf[AT24CS32_PAGE_SIZE];
/* Page2~Page8 数据缓冲（224B，包含每页最后2字节页和） */
static uint8_t s_auth_data[AT24CS32_AUTH_DATA_LENGTH];
/* CRC输入缓冲：SN(16B) + Page2~8(224B) */
static uint8_t s_crc_input[AT24CS32_AUTH_INPUT_LENGTH];

/* 按总线做原始整页读取（不执行页和校验，用于区分读失败与页和失败） */
static uint8_t AT24CS32_ReadPageRaw_ByBus(uint8_t use_i2c3, uint16_t page_index, uint8_t *page_buf)
{
    uint16_t addr;

    if ((page_buf == NULL) || (page_index >= AT24CS32_PAGE_COUNT)) {
        return 0U;
    }

    addr = (uint16_t)(page_index * AT24CS32_PAGE_SIZE);
    return use_i2c3 ?
        AT24CS32_ReadBytes_I2C3(addr, page_buf, AT24CS32_PAGE_SIZE) :
        AT24CS32_ReadBytes_I2C2(addr, page_buf, AT24CS32_PAGE_SIZE);
}

/* 通用 CRC16 计算：初值和多项式由宏配置 */
static uint16_t AT24CS32_Crc16(const uint8_t *data, uint32_t len, uint16_t init, uint16_t poly)
{
    uint16_t crc = init;
    uint32_t i;
    uint8_t j;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0U; j < 8U; j++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/* 按总线读取 Page2~Page8（共7页，224字节），每页读取时都会做页和校验 */
static uint8_t AT24CS32_ReadAuthPages_ByBus(uint8_t use_i2c3, uint8_t *out_buf)
{
    uint8_t page_buf[AT24CS32_PAGE_SIZE];
    uint16_t page_idx;
    uint16_t offset = 0U;

    if (out_buf == NULL) {
        return 0U;
    }

    for (page_idx = AT24CS32_AUTH_PAGE_START_INDEX;
         page_idx < (AT24CS32_AUTH_PAGE_START_INDEX + AT24CS32_AUTH_PAGE_COUNT);
         page_idx++) {
        uint8_t ok = use_i2c3 ?
            AT24CS32_ReadPage_I2C3(page_idx, page_buf) :
            AT24CS32_ReadPage_I2C2(page_idx, page_buf);

        if (ok == 0U) {
            return 0U;
        }

        memcpy(&out_buf[offset], page_buf, AT24CS32_PAGE_SIZE);
        offset = (uint16_t)(offset + AT24CS32_PAGE_SIZE);
    }

    return 1U;
}

/* 按总线执行认证校验：use_i2c3=0 使用 I2C2，use_i2c3=1 使用 I2C3 */
static AT24CS32_CRC_Status AT24CS32_VerifyCrc_ByBus(uint8_t use_i2c3, AT24CS32_CRC_Result *result)
{
    uint16_t crc1;
    uint16_t crc2;
    uint16_t crc3;
    uint16_t crc4;

    if (result == NULL) {
        return AT24CS32_CRC_STATUS_BAD_PARAM;
    }

    memset(result, 0, sizeof(*result));

    /*
     * 先读取 Page1 并验证页和（Page1[30:31]只用于页和校验，不参与认证结果比较）：
     * - 先做原始读取，失败则返回 PAGE1_READ_FAILED
     * - 读取成功后单独校验页和，失败则返回 PAGE1_CHECKSUM_FAILED
     */
    if (AT24CS32_ReadPageRaw_ByBus(use_i2c3, AT24CS32_AUTH_PAGE1_INDEX, s_page1_buf) == 0U) {
        return AT24CS32_CRC_STATUS_PAGE1_READ_FAILED;
    }

    if (AT24CS32_VerifyPageChecksum(s_page1_buf) == 0U) {
        return AT24CS32_CRC_STATUS_PAGE1_CHECKSUM_FAILED;
    }

    memcpy(result->stored_auth, s_page1_buf, AT24CS32_AUTH_RESULT_SIZE);

    /* 读取 SN */
    if (use_i2c3) {
        if (AT24CS32_ReadSerialNumber_I2C3(result->sn) != 0U) {
            return AT24CS32_CRC_STATUS_SN_READ_FAILED;
        }
    } else {
        if (AT24CS32_ReadSerialNumber_I2C2(result->sn) != 0U) {
            return AT24CS32_CRC_STATUS_SN_READ_FAILED;
        }
    }

    /* 读取 Page2~Page8（0x0020~0x00FF）并按页校验 */
    if (AT24CS32_ReadAuthPages_ByBus(use_i2c3, s_auth_data) == 0U) {
        return AT24CS32_CRC_STATUS_DATA_READ_FAILED;
    }

    /* 认证输入为 SN + Page2~8(含每页末尾2字节页和) */
    memcpy(s_crc_input, result->sn, AT24CS32_SN_SIZE);
    memcpy(&s_crc_input[AT24CS32_SN_SIZE], s_auth_data, AT24CS32_AUTH_DATA_LENGTH);

    /* 同一输入做4组CRC16，拼接为8字节（大端） */
    crc1 = AT24CS32_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_1, AT24CS32_CRC16_POLY_1);
    crc2 = AT24CS32_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_2, AT24CS32_CRC16_POLY_2);
    crc3 = AT24CS32_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_3, AT24CS32_CRC16_POLY_3);
    crc4 = AT24CS32_Crc16(s_crc_input, AT24CS32_AUTH_INPUT_LENGTH, AT24CS32_CRC16_INIT_4, AT24CS32_CRC16_POLY_4);

    result->calculated_auth[0] = (uint8_t)(crc1 >> 8);
    result->calculated_auth[1] = (uint8_t)(crc1 & 0xFFU);
    result->calculated_auth[2] = (uint8_t)(crc2 >> 8);
    result->calculated_auth[3] = (uint8_t)(crc2 & 0xFFU);
    result->calculated_auth[4] = (uint8_t)(crc3 >> 8);
    result->calculated_auth[5] = (uint8_t)(crc3 & 0xFFU);
    result->calculated_auth[6] = (uint8_t)(crc4 >> 8);
    result->calculated_auth[7] = (uint8_t)(crc4 & 0xFFU);

    /* 与 Page1 前8字节比对 */
    if (memcmp(result->stored_auth, result->calculated_auth, AT24CS32_AUTH_RESULT_SIZE) != 0) {
        return AT24CS32_CRC_STATUS_CRC_MISMATCH;
    }

    return AT24CS32_CRC_STATUS_OK;
}

/* I2C2 显式接口：按固定布局执行认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C2(AT24CS32_CRC_Result *result)
{
    return AT24CS32_VerifyCrc_ByBus(0U, result);
}

/* I2C3 显式接口：按固定布局执行认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C3(AT24CS32_CRC_Result *result)
{
    return AT24CS32_VerifyCrc_ByBus(1U, result);
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
