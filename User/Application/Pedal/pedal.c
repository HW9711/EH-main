//pedal.c

#include "pedal.h"
#include "uart4.h"
#include "common.h"
#include "delay.h"
#include "Pubinterface.h"
#include "screenkey.h"
#include "kernel_scheduler.h"
#include "kernel_osal.h"

kernel_task_t PEDALRECVTaskHandle;

PedalCalibrationData_t PedalCalibrationData = { 0 };  // 脚踏定标页专用缓存，只保存 UART4 定标回包和按键事件。

#define PEDAL_CALIBRATION_CONNECTED  1U    // 在线标志固定用 1，0 表示离线；这是状态编号，不是可调时间或阈值。
#define PEDAL_CAL_OFFLINE_SCANS       500U  // 连续 500 次扫描无有效数据就清空页面；定标主循环约 2ms 一次，约 1s。调大延后离线清屏，调小更容易因短暂丢包清屏。
#define PEDAL_CAL_SINGLE_FRAME_LEN    10U   // 协议规定的总字节数，含末尾 2 字节 CRC：单踏板、旧读回及按键帧；不可单独改主控值。
#define PEDAL_CAL_TWO_STAGE_FRAME_LEN 18U   // 协议规定双段踏板帧共 18 字节，含实时 AD、三点定标值和 CRC；须与脚踏板协议一致。
#define PEDAL_CAL_DUAL_FRAME_LEN      24U   // 协议规定双脚踏帧共 24 字节，含左右 AD、两组三点值和 CRC；改错会导致整帧校验失败。

/*
 * 函数功能：定标命令发送后等待指定时间，给脚踏板留出处理和回复时间。
 * 输入参数：delay_ms 为本次等待的毫秒数。
 * 返回参数：无。
 */
static void PedalTaskDelayMs(uint32_t delay_ms)
{
  Kernel_DelayUntilMs(delay_ms);  // 标定模式沿用统一调度延时，避免空循环占满CPU。
}

/*
 * 函数功能：清空定标专用脚踏缓存，使新一轮识别不继承历史类型和定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void PedalCal_Reset(void)
{
  Common_Memset(0U, (uint8_t *)&PedalCalibrationData, (uint16_t)sizeof(PedalCalibrationData));  // 一次清除在线、类型、实时值和存储值。
}

static uint8_t Pedal_StorageHDataCMD[8] =       {0xFE, 0xEF, 0xD0, 0xB4, 0xB8, 0xDF, 0x6C, 0x8B}; // 保存单路或左路高点。
static uint8_t Pedal_StorageLDataCMD[8] =       {0xFE, 0xEF, 0xD0, 0xB4, 0xB5, 0xCD, 0xF1, 0x0F}; // 保存单路或左路低点。
static uint8_t Pedal_StorageMDataCMD[8] =       {0xFE, 0xEF, 0xD0, 0xB4, 0xB2, 0xEE, 0x7F, 0x3D}; // 保存左路中点。
static uint8_t Pedal_StorageHDataCMD_Right[8] = {0xFE, 0xEF, 0xD0, 0xB4, 0xB8, 0xDF, 0x6C, 0x9B}; // 保存双脚踏右路高点。
static uint8_t Pedal_StorageLDataCMD_Right[8] = {0xFE, 0xEF, 0xD0, 0xB4, 0xB5, 0xCD, 0xF1, 0x1F}; // 保存双脚踏右路低点。
static uint8_t Pedal_StorageMDataCMD_Right[8] = {0xFE, 0xEF, 0xD0, 0xB4, 0xB2, 0xEE, 0x7F, 0x4D}; // 保存双脚踏右路中点。

static uint8_t Pedal_ReadHDataCMD[8] =       {0xFE, 0xEF, 0xB6, 0xC1, 0xB8, 0xDF, 0x3E, 0x84}; // 请求重新加载单路或左路高点。
static uint8_t Pedal_ReadLDataCMD[8] =       {0xFE, 0xEF, 0xB6, 0xC1, 0xB5, 0xCD, 0xA3, 0x00}; // 请求重新加载单路或左路低点。
static uint8_t Pedal_ReadMDataCMD[8] =       {0xFE, 0xEF, 0xB6, 0xC1, 0xB2, 0xEE, 0x42, 0x29}; // 请求重新加载单路或左路中点。
static uint8_t Pedal_ReadHDataCMD_Right[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB8, 0xDF, 0x3E, 0x94}; // 兼容旧协议的右路高点请求。
static uint8_t Pedal_ReadLDataCMD_Right[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB5, 0xCD, 0xA3, 0x10}; // 兼容旧协议的右路低点请求。
static uint8_t Pedal_ReadMDataCMD_Right[8] = {0xFE, 0xEF, 0xB6, 0xC1, 0xB2, 0xEE, 0x42, 0x28}; // 兼容旧协议的右路中点请求。

/*
 * 函数功能：把当前单踏板或左踏板实时值写为高点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageHValue(void)
{
  Uart4_SendPacket(Pedal_StorageHDataCMD, 8U);  // 发送末字节为 0x8B 的保存命令，让脚踏板保存第一路高点。
}

/*
 * 函数功能：把当前单踏板或左踏板实时值写为低点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageLValue(void)
{
  Uart4_SendPacket(Pedal_StorageLDataCMD, 8U);  // 发送末字节为 0x0F 的保存命令，让脚踏板保存第一路低点。
}

/*
 * 函数功能：把当前左踏板实时值写为中点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageMValue(void)
{
  Uart4_SendPacket(Pedal_StorageMDataCMD, 8U);  // 发送末字节为 0x3D 的保存命令，让脚踏板保存第一路中点。
}

/*
 * 函数功能：把当前双脚踏右路实时值写为高点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageHValue_Right(void)
{
  Uart4_SendPacket(Pedal_StorageHDataCMD_Right, 8U);  // 发送末字节为 0x9B 的命令，让脚踏板保存右路高点。
}

/*
 * 函数功能：把当前双脚踏右路实时值写为低点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageLValue_Right(void)
{
  Uart4_SendPacket(Pedal_StorageLDataCMD_Right, 8U);  // 发送末字节为 0x1F 的命令，让脚踏板保存右路低点。
}

/*
 * 函数功能：把当前双脚踏右路实时值写为中点定标值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_StorageMValue_Right(void)
{
  Uart4_SendPacket(Pedal_StorageMDataCMD_Right, 8U);  // 发送末字节为 0x4D 的命令，让脚踏板保存右路中点。
}

/*
 * 函数功能：请求脚踏板重新加载单路或左路高点存储值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadHValue(void)
{
  Uart4_SendPacket(Pedal_ReadHDataCMD, 8U);  // 脚踏板随后通过周期实时帧返回Flash中的定标值。
}

/*
 * 函数功能：请求脚踏板重新加载单路或左路低点存储值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadLValue(void)
{
  Uart4_SendPacket(Pedal_ReadLDataCMD, 8U);  // 单踏板旧协议可返回D0回包，新协议在DD帧中返回。
}

/*
 * 函数功能：请求脚踏板重新加载单路或左路中点存储值。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadMValue(void)
{
  Uart4_SendPacket(Pedal_ReadMDataCMD, 8U);  // 双段或双脚踏的新协议会在后续DD帧中返回三点值。
}

/*
 * 函数功能：发送兼容旧双脚踏协议的右路高点读取命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadHValue_Right(void)
{
  Uart4_SendPacket(Pedal_ReadHDataCMD_Right, 8U);  // 只为兼容旧板保留，新板周期帧已包含右路高点。
}

/*
 * 函数功能：发送兼容旧双脚踏协议的右路低点读取命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadLValue_Right(void)
{
  Uart4_SendPacket(Pedal_ReadLDataCMD_Right, 8U);  // 只为兼容旧板保留，新板周期帧已包含右路低点。
}

/*
 * 函数功能：发送兼容旧双脚踏协议的右路中点读取命令。
 * 输入参数：无。
 * 返回参数：无。
 */
void Pedal_ReadMValue_Right(void)
{
  Uart4_SendPacket(Pedal_ReadMDataCMD_Right, 8U);  // 只为兼容旧板保留，新板周期帧已包含右路中点。
}
/*
 * 函数功能：依次请求脚踏板保存的高点和低点，两条命令间留出回复时间。
 * 输入参数：无。
 * 返回参数：无；收到的数据由接收解析函数写入定标缓存。
 */
void Pedal_ReadStorageHLValue(void)
{
  Pedal_ReadHValue();  // 先请求单路或左路高点。
	PedalTaskDelayMs(50);  // 等待 50ms，避免高点与低点请求紧挨着发送。

  Pedal_ReadLValue();  // 再请求单路或左路低点。
	PedalTaskDelayMs(50);  // 再留 50ms 给脚踏板处理本次命令。
}

//============================================================================
// 接收UART4脚踏定标数据，按协议帧长度解析单踏板、双段踏板和双脚踏。
//============================================================================

/*
 * 函数功能：把协议中高字节在前的两个字节合成一个 16 位数值。
 * 输入参数：data 指向数值的高字节。
 * 返回参数：返回组合后的16位无符号值。
 */
static uint16_t PedalCal_ReadU16(const uint8_t *data)
{
  return ((uint16_t)data[0] << 8) | data[1];  // 脚踏协议统一按高字节在前传输AD和定标值。
}

/*
 * 函数功能：校验一帧脚踏数据尾部的CRC16。
 * 输入参数：frame 指向完整帧；frame_len 为包含两字节CRC的总长度。
 * 返回参数：1表示CRC正确，0表示长度或CRC不正确。
 */
static uint8_t PedalCal_CheckCrc(uint8_t *frame, uint16_t frame_len)
{
  uint16_t received_crc;  // 保存报文尾部携带的CRC，便于与本机计算值比较。

  if (frame_len < 2U)
  {
    return 0U;  // 长度不足以容纳CRC时禁止继续访问尾部字节。
  }

  received_crc = ((uint16_t)frame[frame_len - 2U] << 8) | frame[frame_len - 1U];  // 协议CRC高字节先发送。
  return (Common_Crc16(frame, (uint16_t)(frame_len - 2U)) == received_crc) ? 1U : 0U;  // 只有完整帧校验通过才进入定标状态。
}

/*
 * 函数功能：记录本次收到的脚踏类型并刷新定标模式在线状态。
 * 输入参数：pedal_type 为 PEDAL_CAL_TYPE_* 定义的脚踏类型。
 * 返回参数：无。
 */
static void PedalCal_MarkData(uint8_t pedal_type)
{
  PedalCalibrationData.FootPedalType = pedal_type;  // 类型决定页面允许保存两点、三点还是左右两组三点。
  PedalCalibrationData.FootPedalConnectFlag = PEDAL_CALIBRATION_CONNECTED;  // 合法实时帧确认脚踏当前在线。
  PedalCalibrationData.FootPedalOffTimes = 0U;  // 收到可信数据后重新开始约1秒离线计时。
}

/*
 * 函数功能：解析10字节单踏板实时帧、旧双踏板实时帧和旧存储值回包。
 * 输入参数：frame 指向已经通过CRC校验的10字节完整帧。
 * 返回参数：1 表示已识别并保存帧内数据，0 表示不支持这条单路协议帧。
 */
static uint8_t PedalCal_ParseSingle(uint8_t *frame)
{
  uint16_t value;  // 保存当前帧携带的实时值或Flash定标值。

  if ((frame[2] == 0xB6U) && (frame[3] == 0xC1U) &&
      (frame[4] == 0x01U) && (frame[5] == 0x01U))
  {
    PedalCalibrationData.FootPedalADValue_Left = PedalCal_ReadU16(&frame[6]);  // 单踏板在页面左区显示实时值。
    PedalCalibrationData.FootPedalADValue_Right = 0U;  // 单踏板没有右路，清零可避免遗留双脚踏显示。
    PedalCal_MarkData(PEDAL_CAL_TYPE_SINGLE);  // 两点保存按键只允许写第一路低点和高点。
    return 1U;
  }

  if ((frame[2] == 0xB6U) && (frame[3] == 0xC1U) &&
      (frame[4] == 0x01U) && (frame[5] == 0x0AU))
  {
    PedalCalibrationData.FootPedalADValue_Left = PedalCal_ReadU16(&frame[6]);  // 兼容旧双脚踏左路分帧上报。
    PedalCal_MarkData(PEDAL_CAL_TYPE_DUAL);  // 分帧旧协议仍按双脚踏开放左右三点保存。
    return 1U;
  }

  if ((frame[2] == 0xB6U) && (frame[3] == 0xC1U) &&
      (frame[4] == 0x01U) && (frame[5] == 0x0BU))
  {
    PedalCalibrationData.FootPedalADValue_Right = PedalCal_ReadU16(&frame[6]);  // 兼容旧双脚踏右路分帧上报。
    PedalCal_MarkData(PEDAL_CAL_TYPE_DUAL);  // 保持已识别的双脚踏类型和在线状态。
    return 1U;
  }

  if ((frame[2] != 0xD0U) || (frame[3] != 0xB4U) ||
      (PedalCalibrationData.FootPedalType == PEDAL_CAL_TYPE_NONE))
  {
    return 0U;  // 未识别类型前不猜测D0回包属于哪一路，防止把旧数据写错显示区。
  }

  value = PedalCal_ReadU16(&frame[6]);  // D0回包的第6、7字节为脚踏Flash中的定标值。
  if (PedalCalibrationData.FootPedalType == PEDAL_CAL_TYPE_SINGLE)
  {
    if ((frame[4] == 0xB5U) && (frame[5] == 0xCDU))
    {
      PedalCalibrationData.FootPedalMemoryLValue_Left = value;  // 单踏板低点回包刷新左侧低值。
    }
    else if ((frame[4] == 0xB8U) && (frame[5] == 0xDFU))
    {
      PedalCalibrationData.FootPedalMemoryHValue_Left = value;  // 单踏板高点回包刷新左侧高值。
    }
    else if ((frame[4] == 0xB2U) && (frame[5] == 0xEEU))
    {
      PedalCalibrationData.FootPedalMemoryMValue_Left = value;  // 兼容旧板返回中点，但单踏板页面不开放保存。
    }
    else
    {
      return 0U;  // 未定义的D0参数码不刷新在线状态。
    }
  }
  else
  {
    if ((frame[4] == 0xB5U) && (frame[5] == 0xCDU))
    {
      PedalCalibrationData.FootPedalMemoryLValue_Right = value;  // 保留旧协议B5表示右路低点的映射。
    }
    else if ((frame[4] == 0xB6U) && (frame[5] == 0xCDU))
    {
      PedalCalibrationData.FootPedalMemoryLValue_Left = value;  // 保留旧协议B6表示左路低点的映射。
    }
    else if ((frame[4] == 0xB8U) && (frame[5] == 0xDFU))
    {
      PedalCalibrationData.FootPedalMemoryHValue_Right = value;  // 保留旧协议B8表示右路高点的映射。
    }
    else if ((frame[4] == 0xB9U) && (frame[5] == 0xDFU))
    {
      PedalCalibrationData.FootPedalMemoryHValue_Left = value;  // 保留旧协议B9表示左路高点的映射。
    }
    else if ((frame[4] == 0xB2U) && (frame[5] == 0xEEU))
    {
      PedalCalibrationData.FootPedalMemoryMValue_Right = value;  // 保留旧协议B2表示右路中点的映射。
    }
    else if ((frame[4] == 0xB3U) && (frame[5] == 0xEEU))
    {
      PedalCalibrationData.FootPedalMemoryMValue_Left = value;  // 保留旧协议B3表示左路中点的映射。
    }
    else
    {
      return 0U;  // 未定义的D0参数码不改变定标缓存。
    }
  }

  PedalCalibrationData.FootPedalConnectFlag = PEDAL_CALIBRATION_CONNECTED;  // 合法存储值回包也可证明脚踏仍在线。
  PedalCalibrationData.FootPedalOffTimes = 0U;  // 旧协议读回过程同样刷新离线计时。
  return 1U;
}
/*
 * 函数功能：把脚踏实体按键码转换为定标页使用的左、中、右事件。
 * 输入参数：frame 指向已经通过CRC校验的10字节按键帧。
 * 返回参数：1表示按键码有效，0表示按键码未定义或当前电机正在运行。
 */
static uint8_t PedalCal_ParseKey(uint8_t *frame)
{
  uint8_t key_value = 0U;  // 页面统一使用的按键编号：1 左、2 中、3 右，0 表示未识别。

  if (WorkMessage.runflag_work != false)
  {
    return 0U;  // 电机运行时不允许脚踏实体键改变定标页面状态。
  }

  if (frame[5] == 0xDDU)
  {
    if (frame[7] == 0x0AU)
    {
      key_value = 1U;  // 兼容旧协议的左键码。
    }
    else if (frame[7] == 0x05U)
    {
      key_value = 2U;  // 兼容旧协议的中键码。
    }
    else if (frame[7] == 0x0BU)
    {
      key_value = 3U;  // 兼容旧协议的右键码。
    }
  }
  else
  {
    if ((frame[7] == 0x01U) || (frame[7] == 0x05U))
    {
      key_value = 1U;  // 新协议左键长按和短按都点亮左键调试状态。
    }
    else if ((frame[7] == 0x03U) || (frame[7] == 0x06U))
    {
      key_value = 2U;  // 新协议中键长按和短按都点亮中键调试状态。
    }
    else if ((frame[7] == 0x02U) || (frame[7] == 0x04U))
    {
      key_value = 3U;  // 新协议右键长按和短按都点亮右键调试状态。
    }
  }

  if (key_value == 0U)
  {
    return 0U;  // 不认识的按键码不更新页面，也不算作一次有效的在线数据。
  }

  PedalCalibrationData.FootPedalKeyValue = key_value;  // 保存最近一次实体按键供定标页调试显示。
  if (key_value == 1U)
  {
    ScreenKey_LegacyEventPost(L_KEY_FOOT);  // 通知定标页左键按过一次，页面取出后会清除此事件。
  }
  else if (key_value == 2U)
  {
    ScreenKey_LegacyEventPost(M_KEY_FOOT);  // 通知定标页中键按过一次。
  }
  else
  {
    ScreenKey_LegacyEventPost(R_KEY_FOOT);  // 通知定标页右键按过一次。
  }

  PedalCalibrationData.FootPedalConnectFlag = PEDAL_CALIBRATION_CONNECTED;  // 合法按键帧证明UART4链路当前在线。
  PedalCalibrationData.FootPedalOffTimes = 0U;  // 按键活动同样重新开始离线计时。
  return 1U;
}

/*
 * 函数功能：解析双段踏板、双脚踏实时帧及脚踏实体按键帧。
 * 输入参数：frame 指向已经通过CRC校验的完整帧。
 * 返回参数：1 表示已识别并处理本帧，0 表示功能或脚踏类型不支持。
 */
static uint8_t PedalCal_ParseMulti(uint8_t *frame)
{
  if ((frame[2] != 0xBBU) || (frame[3] != 0xAAU))
  {
    return 0U;  // 只处理当前脚踏板定义的BB AA实时值和按键协议。
  }

  if ((frame[4] == 0xDDU) && (frame[5] == 0x01U))
  {
    PedalCalibrationData.FootPedalADValue_Left = PedalCal_ReadU16(&frame[6]);  // 双段踏板只有一组实时AD。
    PedalCalibrationData.FootPedalADValue_Right = 0U;  // 切换到双段踏板时清除历史右路显示。
    PedalCalibrationData.FootPedalMemoryHValue_Left = PedalCal_ReadU16(&frame[10]);  // 帧内同步返回左路高点Flash值。
    PedalCalibrationData.FootPedalMemoryMValue_Left = PedalCal_ReadU16(&frame[12]);  // 帧内同步返回左路中点Flash值。
    PedalCalibrationData.FootPedalMemoryLValue_Left = PedalCal_ReadU16(&frame[14]);  // 帧内同步返回左路低点Flash值。
    PedalCalibrationData.FootPedalMemoryHValue_Right = 0U;  // 非双脚踏不显示历史右路高点。
    PedalCalibrationData.FootPedalMemoryMValue_Right = 0U;  // 非双脚踏不显示历史右路中点。
    PedalCalibrationData.FootPedalMemoryLValue_Right = 0U;  // 非双脚踏不显示历史右路低点。
    PedalCal_MarkData(PEDAL_CAL_TYPE_TWO_STAGE);  // 三点保存只开放页面左侧低、中、高按钮。
    return 1U;
  }

  if ((frame[4] == 0xDDU) && (frame[5] == 0x02U))
  {
    PedalCalibrationData.FootPedalADValue_Left = PedalCal_ReadU16(&frame[6]);  // 双脚踏左路实时AD。
    PedalCalibrationData.FootPedalADValue_Right = PedalCal_ReadU16(&frame[8]);  // 双脚踏右路实时AD。
    PedalCalibrationData.FootPedalMemoryHValue_Left = PedalCal_ReadU16(&frame[10]);  // 左路高点Flash值。
    PedalCalibrationData.FootPedalMemoryMValue_Left = PedalCal_ReadU16(&frame[12]);  // 左路中点Flash值。
    PedalCalibrationData.FootPedalMemoryLValue_Left = PedalCal_ReadU16(&frame[14]);  // 左路低点Flash值。
    PedalCalibrationData.FootPedalMemoryHValue_Right = PedalCal_ReadU16(&frame[16]);  // 右路高点Flash值。
    PedalCalibrationData.FootPedalMemoryMValue_Right = PedalCal_ReadU16(&frame[18]);  // 右路中点Flash值。
    PedalCalibrationData.FootPedalMemoryLValue_Right = PedalCal_ReadU16(&frame[20]);  // 右路低点Flash值。
    PedalCal_MarkData(PEDAL_CAL_TYPE_DUAL);  // 左右两组三点保存按钮全部开放。
    return 1U;
  }

  if (frame[4] == 0xCCU)
  {
    return PedalCal_ParseKey(frame);  // 按键帧不包含定标值，只更新页面实体键调试状态。
  }

  return 0U;  // 未定义功能码不改变定标缓存。
}
/*
 * 函数功能：依据脚踏协议头和类型字段确定当前完整帧长度。
 * 输入参数：frame 指向FE EF帧头；remaining 为当前缓冲区从帧头起的剩余长度。
 * 返回参数：返回10、18或24；字段不足或协议不支持时返回0。
 */
static uint16_t PedalCal_GetLength(const uint8_t *frame, uint16_t remaining)
{
  if (remaining < 6U)
  {
    return 0U;  // 长度不足时不能读取协议类型字段。
  }

  if (((frame[2] == 0xB6U) && (frame[3] == 0xC1U)) ||
      ((frame[2] == 0xD0U) && (frame[3] == 0xB4U)))
  {
    return PEDAL_CAL_SINGLE_FRAME_LEN;  // 单路实时值和旧存储值回包均为10字节。
  }

  if ((frame[2] == 0xBBU) && (frame[3] == 0xAAU))
  {
    if ((frame[4] == 0xDDU) && (frame[5] == 0x01U))
    {
      return PEDAL_CAL_TWO_STAGE_FRAME_LEN;  // 双段踏板带一组三点Flash值。
    }
    if ((frame[4] == 0xDDU) && (frame[5] == 0x02U))
    {
      return PEDAL_CAL_DUAL_FRAME_LEN;  // 双脚踏带左右两组三点Flash值。
    }
    if (frame[4] == 0xCCU)
    {
      return PEDAL_CAL_SINGLE_FRAME_LEN;  // 新旧脚踏按键帧均保持10字节。
    }
  }

  return 0U;  // 未识别协议头时继续向后寻找下一个FE EF。
}

/*
 * 函数功能：在标定模式下维护约1秒的脚踏离线状态。
 * 输入参数：valid_frame 非0表示本周期至少收到一帧合法且支持的脚踏数据。
 * 返回参数：无。
 */
static void PedalCal_ServiceOffline(uint8_t valid_frame)
{
  if (valid_frame != 0U)
  {
    PedalCalibrationData.FootPedalOffTimes = 0U;  // 合法帧到达后取消离线累计。
    return;
  }

  if (PedalCalibrationData.FootPedalOffTimes < PEDAL_CAL_OFFLINE_SCANS)
  {
    PedalCalibrationData.FootPedalOffTimes++;  // 标定主循环每2ms调用一次，累计到500约为1秒。
  }

  if ((PedalCalibrationData.FootPedalOffTimes >= PEDAL_CAL_OFFLINE_SCANS) &&
      ((PedalCalibrationData.FootPedalConnectFlag != 0U) ||
       (PedalCalibrationData.FootPedalType != PEDAL_CAL_TYPE_NONE)))
  {
    PedalCal_Reset();  // 离线时清除实时值、Flash值和类型，避免页面继续显示已经拔出的脚踏。
    PedalCalibrationData.FootPedalOffTimes = PEDAL_CAL_OFFLINE_SCANS;  // 计数停在上限，离线后不反复清零和累加。
  }
}

/*
 * 函数功能：扫描UART4 DMA缓冲区并解析全部完整、CRC正确的脚踏定标帧。
 * 输入参数：无，直接取出 UART4 DMA 已收到的数据。
 * 返回参数：无；结果写入 PedalCalibrationData，脚踏按键交给定标页处理。
 */
void PedalRecv_Scan(void)
{
  uint8_t data[UART4_MAX_PACKET_SIZE] = { 0U };  // 保存本次从 UART4 DMA 取出的接收数据。
  uint16_t data_len;  // 本次接收数据的有效字节数。
  uint16_t offset = 0U;  // 当前搜索帧头的位置。
  uint16_t frame_len;  // 当前协议帧按类型确定的完整长度。
  uint8_t valid_frame = 0U;  // 非 0 表示本次至少处理过一帧有效脚踏数据，可清除离线计数。

  data_len = Uart4_DMARecvDataPeek(data);  // 读取脚踏串口数据，不直接复用正常业务任务的解析状态。
  while ((offset + 1U) < data_len)
  {
    if ((data[offset] != 0xFEU) || (data[offset + 1U] != 0xEFU))
    {
      offset++;  // 丢弃帧头前噪声并继续寻找下一组FE EF。
      continue;
    }

    frame_len = PedalCal_GetLength(&data[offset], (uint16_t)(data_len - offset));  // 按类型选择10、18或24字节帧。
    if (frame_len == 0U)
    {
      offset++;  // 未识别的FE EF不能阻塞后续合法帧搜索。
      continue;
    }

    if ((uint16_t)(data_len - offset) < frame_len)
    {
      break;  // 本批数据末尾不足一帧，停止本次解析，不读取缓冲区之外的数据。
    }

    if (PedalCal_CheckCrc(&data[offset], frame_len) == 0U)
    {
      offset++;  // CRC 错误时向后一字节继续找帧头，后面仍可能有正确的数据帧。
      continue;
    }

    if ((frame_len == PEDAL_CAL_SINGLE_FRAME_LEN) &&
        ((data[offset + 2U] == 0xB6U) || (data[offset + 2U] == 0xD0U)))
    {
      valid_frame |= PedalCal_ParseSingle(&data[offset]);  // 解析单踏板或旧兼容分帧协议。
    }
    else
    {
      valid_frame |= PedalCal_ParseMulti(&data[offset]);  // 解析双段、双脚踏或实体按键帧。
    }

    offset = (uint16_t)(offset + frame_len);  // 本帧已处理，直接从它后面开始找下一帧。
  }

  PedalCal_ServiceOffline(valid_frame);  // 本周期没有合法帧时推进离线计时。
}
/* USER CODE BEGIN Header_PEDALRECVTaskFunc */
/**
* @brief Function implementing the PEDALRECVTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PEDALRECVTaskFunc */
/*
 * 函数功能：周期检查脚踏 UART4 定标回包，并在外控占用时暂停本地脚踏数据处理。
 * 输入参数：event 为调度器事件参数，当前任务不使用该值。
 * 返回参数：无。
 */
void PEDALRECVTaskFunc(uint32_t event)
{
  /* USER CODE BEGIN PEDALRECVTaskFunc */
  /* Infinite loop */
	/* 外控占用期间暂停本地脚踏定标解析，避免脚踏页面状态与外控运行状态同时变化。 */
	if(WorkMessage.hmiactive_work)
		return;
		PedalRecv_Scan();
  /* USER CODE END PEDALRECVTaskFunc */
}

/*
 * 函数功能：创建脚踏定标接收任务并设置 3ms 周期；启动定标页面也可自行调用 PedalRecv_Scan。
 * 输入参数：无。
 * 返回参数：无。
 */
void PedalRecvTask_Init(void)
{
  /* definition and creation of PEDALRECVTask */
	Kernel_TaskCreate(&PEDALRECVTaskHandle, PEDALRECVTaskFunc);  // 注册接收处理函数，供调度器调用。
	Kernel_TaskStart(&PEDALRECVTaskHandle, KERNEL_TASK_ALWAYS, 3);  // 若使用此任务，每 3ms 扫描一次，500 次无数据约为 1.5s。
}









