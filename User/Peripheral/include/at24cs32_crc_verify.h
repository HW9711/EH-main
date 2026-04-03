#ifndef __AT24CS32_CRC_VERIFY_H__
#define __AT24CS32_CRC_VERIFY_H__

#include "at24cs32.h"
#include <stdint.h>

/*
 * AT24CS32 手柄认证校验说明：
 * 1) 读取 SN（16B）
 * 2) 读取 Page2~Page8（0x0020~0x00FF，共224B，包含各页末尾2字节页和）
 * 3) 基于同一输入执行 4 组 CRC16，拼接得到 8 字节认证结果
 * 4) 与 Page1[0:7] 存储认证结果比较
 * 5) Page1[30:31] 仅用于页和校验，不参与认证结果比较
 */

#ifdef __cplusplus
extern "C" {
#endif

#define AT24CS32_AUTH_RESULT_SIZE            8U
#define AT24CS32_AUTH_PAGE1_INDEX            0U
#define AT24CS32_AUTH_PAGE_START_INDEX       1U   /* Page2 对应页索引 1 */
#define AT24CS32_AUTH_PAGE_COUNT             7U   /* Page2~Page8 共7页 */
#define AT24CS32_AUTH_DATA_START_ADDR        0x0020U
#define AT24CS32_AUTH_DATA_LENGTH            224U
#define AT24CS32_AUTH_INPUT_LENGTH           (AT24CS32_SN_SIZE + AT24CS32_AUTH_DATA_LENGTH)

/* 认证使用4组CRC16，默认初值均为0xFFFF，支持宏覆盖 */
#ifndef AT24CS32_CRC16_INIT_1
#define AT24CS32_CRC16_INIT_1                0xFFFFU
#endif
#ifndef AT24CS32_CRC16_INIT_2
#define AT24CS32_CRC16_INIT_2                0xFFFFU
#endif
#ifndef AT24CS32_CRC16_INIT_3
#define AT24CS32_CRC16_INIT_3                0xFFFFU
#endif
#ifndef AT24CS32_CRC16_INIT_4
#define AT24CS32_CRC16_INIT_4                0xFFFFU
#endif

#ifndef AT24CS32_CRC16_POLY_1
#define AT24CS32_CRC16_POLY_1                0x1021U
#endif
#ifndef AT24CS32_CRC16_POLY_2
#define AT24CS32_CRC16_POLY_2                0x8005U
#endif
#ifndef AT24CS32_CRC16_POLY_3
#define AT24CS32_CRC16_POLY_3                0x3D65U
#endif
#ifndef AT24CS32_CRC16_POLY_4
#define AT24CS32_CRC16_POLY_4                0xA097U
#endif

typedef enum
{
    AT24CS32_CRC_STATUS_OK = 0,
    AT24CS32_CRC_STATUS_BAD_PARAM,
    AT24CS32_CRC_STATUS_PAGE1_READ_FAILED,
    AT24CS32_CRC_STATUS_PAGE1_CHECKSUM_FAILED,
    AT24CS32_CRC_STATUS_SN_READ_FAILED,
    AT24CS32_CRC_STATUS_DATA_READ_FAILED,
    AT24CS32_CRC_STATUS_CRC_MISMATCH
} AT24CS32_CRC_Status;

typedef struct
{
    uint8_t sn[AT24CS32_SN_SIZE];
    uint8_t stored_auth[AT24CS32_AUTH_RESULT_SIZE];
    uint8_t calculated_auth[AT24CS32_AUTH_RESULT_SIZE];
} AT24CS32_CRC_Result;

/*
 * RTOS推荐接口：显式绑定总线，避免全局句柄切换导致并发竞态
 */
/* 使用 I2C2 执行 Page1+Page2~8 布局认证校验，返回状态码 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C2(AT24CS32_CRC_Result *result);
/* 使用 I2C3 执行 Page1+Page2~8 布局认证校验，返回状态码 */
AT24CS32_CRC_Status AT24CS32_VerifyCrc_I2C3(AT24CS32_CRC_Result *result);

/* 兼容旧命名：默认布局认证校验 */
AT24CS32_CRC_Status AT24CS32_VerifyCrcDefault_I2C2(AT24CS32_CRC_Result *result);
AT24CS32_CRC_Status AT24CS32_VerifyCrcDefault_I2C3(AT24CS32_CRC_Result *result);

#ifdef __cplusplus
}
#endif

#endif
