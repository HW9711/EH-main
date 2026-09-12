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

#define AT24CS32_AUTH_RESULT_SIZE            8U /* 认证结果固定8字节，由4组CRC16拼成；修改会与现有手柄数据不兼容。 */
#define AT24CS32_AUTH_PAGE1_INDEX            0U /* 保存认证结果的Page1，页索引从0开始，不是可调的识别参数。 */
#define AT24CS32_AUTH_PAGE_START_INDEX       1U   /* Page2 对应页索引 1 */
#define AT24CS32_AUTH_PAGE_COUNT             7U   /* Page2~Page8 共7页 */
#define AT24CS32_AUTH_DATA_START_ADDR        0x0020U /* Page2起始字节地址；保留作格式说明，实际读取按页索引计算。 */
#define AT24CS32_AUTH_DATA_LENGTH            224U /* 参与认证的Page2~8共224字节，包含每页末尾校验和。 */
#define AT24CS32_AUTH_INPUT_LENGTH           (AT24CS32_SN_SIZE + AT24CS32_AUTH_DATA_LENGTH) /* CRC输入共240字节：16字节序列号加224字节页数据。 */

/* 下列INIT/POLY按1~4成对设置每组CRC16的初值和计算多项式。
 * 它们决定认证结果；任一值修改都要同步EEPROM写入工具及手柄记录，否则原有手柄会认证失败。
 * 可由编译参数覆盖，不是放宽认证或调整识别速度的开关。 */
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
    AT24CS32_CRC_STATUS_CRC_MISMATCH,
    AT24CS32_CRC_STATUS_PENDING /* 分步认证尚未完成，不计失败、不发布手柄上线。 */
} AT24CS32_CRC_Status;

typedef struct
{
    uint8_t sn[AT24CS32_SN_SIZE];
    uint8_t stored_auth[AT24CS32_AUTH_RESULT_SIZE];
    uint8_t calculated_auth[AT24CS32_AUTH_RESULT_SIZE];
} AT24CS32_CRC_Result;

/* A/B通道各自保存认证进度和页数据，交替读取时不会覆盖另一通道的数据。 */
typedef struct
{
    uint8_t step; /* 0读Page1、1读SN、2~8读Page2~8；新一轮认证必须归零。 */
    uint8_t verified; /* 所有页的校验和及四组CRC全部通过时置1；为0时禁止使用这些参数初始化手柄。 */
    uint16_t crc[4]; /* 保存SN及已读页面的四组累计CRC，算法与同步认证一致。 */
    AT24CS32_CRC_Result result; /* 本轮SN、存储认证值和计算结果，通道间不交叉。 */
    uint8_t pages[AT24CS32_AUTH_DATA_LENGTH]; /* 保存本轮读取的Page2~Page8；认证通过后直接用这些数据初始化手柄，不再重读EEPROM。 */
} AT24CS32_CRC_StepContext;

/* 每次最多读取一页或SN；use_i2c3为0选I2C2、1选I2C3，PENDING时下轮继续。 */
AT24CS32_CRC_Status AT24CS32_VerifyCrcStep(uint8_t use_i2c3, AT24CS32_CRC_StepContext *context);

/*
 * 根据函数名选择I2C2或I2C3，不需要先修改全局总线变量；调用方仍须避免同时使用公共认证缓存。
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
