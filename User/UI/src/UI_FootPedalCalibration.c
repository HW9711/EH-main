//UI_FootPedalCalibration.c

#include "UI_FootPedalCalibration.h"
#include "lcd.h"
#include "delay.h"
#include "pedal.h"
#include "iwdg.h"
#include "screenkey.h"
#include "sscBEEP.h"
#include "screen_address.h"

#define FOOT_CAL_LOOP_DELAY_MS     2U    // 标定专用循环每次等待2ms，兼顾UART4解析和看门狗刷新。
#define FOOT_CAL_REFRESH_TICKS     50U   // 每50个循环约刷新一次Page3，避免连续写屏占满UART6。
#define FOOT_CAL_QUERY_LOW_TICK    100U  // 单踏板识别后分时请求低点Flash值。
#define FOOT_CAL_QUERY_HIGH_TICK   200U  // 低点请求后继续分时请求高点Flash值。
#define FOOT_CAL_QUERY_RESET_TICK  250U  // 一轮查询结束后清零计数，约500ms开始下一轮。

/*
 * 函数功能：向Page3的一个四字节数值VP写入数据，并给屏幕串口留出发送间隔。
 * 输入参数：vp_address 为DWIN变量地址；value 为需要显示的无符号数值。
 * 返回参数：无。
 */
static void FootCal_Show(uint16_t vp_address, uint32_t value)
{
  LCD_Show_4byte_Number(vp_address, value);  // 使用现有DWIN四字节数值协议刷新指定定标字段。
  Delay_ms(2U);  // 相邻VP写入错开2ms，避免启动阶段UART6连续帧粘连。
}

/*
 * 函数功能：按当前脚踏类型刷新Page3实时值、Flash定标值和三枚实体键调试值。
 * 输入参数：left_key、middle_key、right_key 为页面顶部三枚按键的0/1显示值。
 * 返回参数：无。
 */
static void FootCal_Refresh(uint8_t left_key, uint8_t middle_key, uint8_t right_key)
{
  uint32_t left_ad = 0U;    // 页面左侧实时值，未连接时保持为零。
  uint32_t left_low = 0U;   // 页面左侧低点Flash值，按类型选择是否显示。
  uint32_t left_mid = 0U;   // 页面左侧中点Flash值，单踏板不显示。
  uint32_t left_high = 0U;  // 页面左侧高点Flash值，按类型选择是否显示。
  uint32_t right_ad = 0U;   // 页面右侧实时值，仅双脚踏显示。
  uint32_t right_low = 0U;  // 页面右侧低点Flash值，仅双脚踏显示。
  uint32_t right_mid = 0U;  // 页面右侧中点Flash值，仅双脚踏显示。
  uint32_t right_high = 0U; // 页面右侧高点Flash值，仅双脚踏显示。

  if (PedalCalibrationData.FootPedalConnectFlag != 0U)
  {
    left_ad = PedalCalibrationData.FootPedalADValue_Left;  // 三类脚踏都把主实时值显示在左侧区域。
    left_low = PedalCalibrationData.FootPedalMemoryLValue_Left;  // 三类脚踏都使用第一路低点。
    left_high = PedalCalibrationData.FootPedalMemoryHValue_Left;  // 三类脚踏都使用第一路高点。

    if ((PedalCalibrationData.FootPedalType == PEDAL_CAL_TYPE_TWO_STAGE) ||
        (PedalCalibrationData.FootPedalType == PEDAL_CAL_TYPE_DUAL))
    {
      left_mid = PedalCalibrationData.FootPedalMemoryMValue_Left;  // 双段和双脚踏才开放第一路中点。
    }

    if (PedalCalibrationData.FootPedalType == PEDAL_CAL_TYPE_DUAL)
    {
      right_ad = PedalCalibrationData.FootPedalADValue_Right;  // 双脚踏显示第二路实时值。
      right_low = PedalCalibrationData.FootPedalMemoryLValue_Right;  // 双脚踏显示第二路低点。
      right_mid = PedalCalibrationData.FootPedalMemoryMValue_Right;  // 双脚踏显示第二路中点。
      right_high = PedalCalibrationData.FootPedalMemoryHValue_Right;  // 双脚踏显示第二路高点。
    }
  }

  FootCal_Show(UIDP_LCD_VP_PEDAL_LOW_KEY_VALUE, left_key);  // 刷新左实体键调试值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_MID_KEY_VALUE, middle_key);  // 刷新中实体键调试值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_HIGH_KEY_VALUE, right_key);  // 刷新右实体键调试值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_LEFT_AD_VALUE, left_ad);  // 刷新第一路实时AD。
  FootCal_Show(UIDP_LCD_VP_PEDAL_LEFT_LOW_STORE, left_low);  // 刷新第一路低点Flash值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_LEFT_MID_STORE, left_mid);  // 刷新第一路中点Flash值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_LEFT_HIGH_STORE, left_high);  // 刷新第一路高点Flash值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_RIGHT_AD_VALUE, right_ad);  // 刷新第二路实时AD。
  FootCal_Show(UIDP_LCD_VP_PEDAL_RIGHT_LOW_STORE, right_low);  // 刷新第二路低点Flash值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_RIGHT_MID_STORE, right_mid);  // 刷新第二路中点Flash值。
  FootCal_Show(UIDP_LCD_VP_PEDAL_RIGHT_HIGH_STORE, right_high);  // 刷新第二路高点Flash值。
}

/*
 * 函数功能：为Page3保存按钮提供100ms按键音，并按已识别脚踏类型发送对应的Flash写命令。
 * 输入参数：key_value 为ScreenKey模块投递的一次性保存事件。
 * 返回参数：无；非保存事件不响，类型不支持或脚踏离线时只响不写。
 */
static void FootCal_WriteKey(uint8_t key_value)
{
  uint8_t command_sent = 0U;  // 记录本次是否实际发送写命令，用于统一保留UART4处理间隔。
  uint8_t pedal_type = PedalCalibrationData.FootPedalType;  // 固定本次判断使用的脚踏类型快照。

  if ((key_value != KEY_STORAGEMIN) &&
      (key_value != KEY_STORAGEMAX) &&
      (key_value != KEY_STORAGEMIN2) &&
      (key_value != KEY_STORAGEMAX2) &&
      (key_value != KEY_STORAMEDIAN) &&
      (key_value != KEY_STORAMEDIAN2))
  {
    return;  // KEY_NONE、脚踏实体键及未定义事件不产生定标保存按键音。
  }

  Beep_Pulse100ms();  // 六个Page3保存按钮先给出100ms触摸反馈，脚踏离线或类型不支持时仍不写Flash。

  if (PedalCalibrationData.FootPedalConnectFlag == 0U)
  {
    return;  // 脚踏离线时禁止保存，避免用户误以为零值已经写入Flash。
  }

  switch (key_value)
  {
    case KEY_STORAGEMIN:
      if ((pedal_type == PEDAL_CAL_TYPE_SINGLE) ||
          (pedal_type == PEDAL_CAL_TYPE_TWO_STAGE) ||
          (pedal_type == PEDAL_CAL_TYPE_DUAL))
      {
        Pedal_StorageLValue();  // 第一组命令把当前单路或左路AD写入低点Flash。
        command_sent = 1U;  // 标记已发送，退出switch后给脚踏板留出处理时间。
      }
      break;

    case KEY_STORAGEMAX:
      if ((pedal_type == PEDAL_CAL_TYPE_SINGLE) ||
          (pedal_type == PEDAL_CAL_TYPE_TWO_STAGE) ||
          (pedal_type == PEDAL_CAL_TYPE_DUAL))
      {
        Pedal_StorageHValue();  // 第一组命令把当前单路或左路AD写入高点Flash。
        command_sent = 1U;
      }
      break;

    case KEY_STORAMEDIAN:
      if ((pedal_type == PEDAL_CAL_TYPE_TWO_STAGE) ||
          (pedal_type == PEDAL_CAL_TYPE_DUAL))
      {
        Pedal_StorageMValue();  // 单踏板不支持中点，只有三点类型才写第一路中点。
        command_sent = 1U;
      }
      break;

    case KEY_STORAGEMIN2:
      if (pedal_type == PEDAL_CAL_TYPE_DUAL)
      {
        Pedal_StorageLValue_Right();  // 第二组命令把双脚踏右路AD写入低点Flash。
        command_sent = 1U;
      }
      break;

    case KEY_STORAGEMAX2:
      if (pedal_type == PEDAL_CAL_TYPE_DUAL)
      {
        Pedal_StorageHValue_Right();  // 第二组命令把双脚踏右路AD写入高点Flash。
        command_sent = 1U;
      }
      break;

    case KEY_STORAMEDIAN2:
      if (pedal_type == PEDAL_CAL_TYPE_DUAL)
      {
        Pedal_StorageMValue_Right();  // 第二组命令把双脚踏右路AD写入中点Flash。
        command_sent = 1U;
      }
      break;

    default:
      break;  // 实体键调试事件和未定义事件不触发Flash写入。
  }

  if (command_sent != 0U)
  {
    Delay_ms(5U);  // 保留旧标定流程的5ms写命令间隔，不在主控端伪造保存成功。
  }
}
/*
 * 函数功能：为不在周期帧中携带Flash值的单踏板分时发送低点和高点读取命令。
 * 输入参数：query_ticks 指向本轮单踏板查询计数。
 * 返回参数：无。
 */
static void FootCal_QuerySingle(uint16_t *query_ticks)
{
  if ((PedalCalibrationData.FootPedalConnectFlag == 0U) ||
      (PedalCalibrationData.FootPedalType != PEDAL_CAL_TYPE_SINGLE))
  {
    *query_ticks = 0U;  // 非单踏板或离线时不发送兼容读取命令。
    return;
  }

  (*query_ticks)++;  // 标定循环每2ms推进一次，查询命令按固定时点错开发送。
  if (*query_ticks == FOOT_CAL_QUERY_LOW_TICK)
  {
    Pedal_ReadLValue();  // 先请求单踏板低点，等待D0回包更新页面缓存。
  }
  else if (*query_ticks == FOOT_CAL_QUERY_HIGH_TICK)
  {
    Pedal_ReadHValue();  // 再请求单踏板高点，避免两次应答在UART4中重叠。
  }
  else if (*query_ticks >= FOOT_CAL_QUERY_RESET_TICK)
  {
    *query_ticks = 0U;  // 完成一轮后重新计时，持续显示脚踏板Flash真实值。
  }
}

/*
 * 函数功能：在启动阶段运行Page3脚踏定标模式，处理屏幕保存键、脚踏协议和数值显示。
 * 输入参数：无。
 * 返回参数：无；进入该模式后保持到设备重新上电。
 */
void UI_FootPedalCalibration_Fun(void)
{
  uint8_t left_key = 0U;  // 左实体键调试显示值，每次合法事件在0和1之间切换。
  uint8_t middle_key = 0U;  // 中实体键调试显示值。
  uint8_t right_key = 0U;  // 右实体键调试显示值。
  uint8_t key_value;  // 保存ScreenKey模块投递的一次性事件。
  uint16_t refresh_ticks = 0U;  // 控制Page3刷新频率，避免每2ms连续写屏。
  uint16_t query_ticks = 0U;  // 控制单踏板低点和高点读取命令的分时发送。

  PedalCal_Reset();  // 进入标定模式前清空历史类型和值，必须由当前脚踏合法帧重新识别。
  FootCal_Refresh(0U, 0U, 0U);  // 首次进入Page3立即把所有数值VP清零，避免显示屏保留上次数据。

  for (;;)
  {
    Delay_ms(FOOT_CAL_LOOP_DELAY_MS);  // 保持标定循环节拍并让UART DMA接收新数据。
    ScreenKey_Scan();  // 解析Page3触控保存键和启动阶段屏幕事件。
    PedalRecv_Scan();  // 解析脚踏10、18、24字节协议并更新定标缓存。
    Iwdg_Reset();  // 标定模式仍持续刷新硬件看门狗，避免长期停留触发复位。

    key_value = ScreenKey_LegacyEventTake();  // 每个屏幕或脚踏事件只消费一次。
    if (key_value == L_KEY_FOOT)
    {
      left_key ^= 1U;  // 左实体键事件只切换顶部调试显示，不写Flash。
    }
    else if (key_value == M_KEY_FOOT)
    {
      middle_key ^= 1U;  // 中实体键事件只切换顶部调试显示。
    }
    else if (key_value == R_KEY_FOOT)
    {
      right_key ^= 1U;  // 右实体键事件只切换顶部调试显示。
    }
    else
    {
      FootCal_WriteKey(key_value);  // 其它事件按脚踏类型判断是否允许保存定标点。
    }

    FootCal_QuerySingle(&query_ticks);  // 只有单踏板需要额外分时读取低点和高点Flash值。

    refresh_ticks++;  // 累计Page3刷新节拍。
    if (refresh_ticks >= FOOT_CAL_REFRESH_TICKS)
    {
      refresh_ticks = 0U;  // 一轮显示完成后重新计数。
      FootCal_Refresh(left_key, middle_key, right_key);  // 页面只显示脚踏合法回包确认的实时值和Flash值。
    }
  }
}