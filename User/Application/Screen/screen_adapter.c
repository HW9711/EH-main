#include "screen.h"

#include "Pubinterface.h"
#include "data.h"
#include "lcd.h"

#include <stddef.h>

uint32_t paoxueSpeciValue_A[4] = {0U};
uint32_t paoxueSpeciValue_B[4] = {0U};
uint8_t paoxueSpeciValue_F[16] = {0U};
volatile uint8_t KeyBeep_flag = 0U;

/*
 * 历史屏幕模块的按键分发、蜂鸣和数据仓库职责已经由 Pubinterface/ssc* 接管。
 * 本适配层只保留外部模块仍调用的 LCD 绘图入口，避免 UI_Main、Warn、param 等
 * 既有路径被迫继续链接历史屏幕全局数据结构。
 */
static uint8_t s_info_a = 0U;
static uint8_t s_info_b = 0U;
static uint8_t s_info_hz = 0U;
static uint8_t s_info_extra = 0U;
static uint8_t s_handle_order[5] = {0U};

static uint8_t Screen_PumpGear_FromValue(uint16_t value, uint16_t step, uint16_t max)
{
    uint8_t gear = 0U;

    if (value == 0U)
    {
        return 0U;
    }

    gear = (uint8_t)((value + step - 1U) / step);
    if (gear > 10U)
    {
        gear = 10U;
    }

    if (value > max)
    {
        gear = 10U;
    }

    return gear;
}

static void Screen_ShowSimpleHandle(uint16_t addr, uint8_t connected, uint8_t selected)
{
    if (connected == 0U)
    {
        LCD_Show_Picture(addr, (addr == 0x1500U) ? 106U : 206U);
    }
    else
    {
        LCD_Show_Picture(addr, (uint16_t)((addr == 0x1500U ? 100U : 200U) + (selected ? 4U : 1U)));
    }
}

void PoweronInit(void)
{
    LCD_Show_Which_Map(4U);
    speeddisplay(0U, 0U);
    freqdisplay(1U, 0U);
    injectiondisplay(0U, 0U);
    Infusiondisplay(0U, 0U);
    draindisplay(0U);
    Screen_TipInfo_Update(0U);
}

void ssc_Connectfootpedal(uint8_t ui_data)
{
    Screen_FootPedalConnectState_Update(ui_data);
}

void ssc_Connecthandel(uint8_t ui_data)
{
    Screen_FootPedalConnectState_Update(ui_data);
}

void ssc_Connecttouch(uint8_t ui_data)
{
    (void)ui_data;
}

void Ahandeldisplay(uint8_t ui_data)
{
    LCD_Show_Picture(0x1401U, (uint16_t)(100U + ui_data));
}

void Bhandeldisplay(uint8_t ui_data)
{
    LCD_Show_Picture(0x1402U, (uint16_t)(200U + ui_data));
}

void speeddisplay(uint8_t ui_data, uint32_t speed_data)
{
    if (ui_data != 0U)
    {
        LCD_Show_Picture(0x1407U, 341U);
        LCD_Show_Number(0x9420U, 0x3420U);
        LCD_Show_4byte_Number(0x3420U, speed_data);
    }
    else
    {
        LCD_Show_Picture(0x1407U, 340U);
        LCD_Disappear_Number(0x9420U);
    }
}

void freqdisplay(uint8_t ui_hide_flag, uint8_t fre_data)
{
    if (ui_hide_flag != 0U)
    {
        LCD_Disappear_Picture(0x1411U);
        LCD_Disappear_Number(0x9470U);
    }
    else
    {
        LCD_Show_Picture(0x1411U, 351U);
        LCD_Show_Number(0x9470U, 0x3470U);
        LCD_Show_4byte_Number(0x3470U, fre_data);
    }
}

void injectiondisplay(uint8_t activation_flag, uint16_t inject_data)
{
    if (activation_flag != 0U)
    {
        LCD_Show_Picture(0x1601U, 483U);
        LCD_Show_Picture(0x1603U, 499U);
        LCD_Show_Picture(0x1604U, 501U);
        LCD_Show_Picture(0x1605U, 481U);
        LCD_Show_Number(0x9530U, 0x3530U);
        LCD_Show_4byte_Number(0x3530U, inject_data);
        LCD_Show_Picture(0x1602U, (uint16_t)(487U + Screen_PumpGear_FromValue(inject_data, 5U, 70U)));
    }
    else
    {
        LCD_Show_Picture(0x1601U, 482U);
        LCD_Show_Picture(0x1603U, 498U);
        LCD_Show_Picture(0x1604U, 500U);
        LCD_Show_Picture(0x1602U, 487U);
        LCD_Show_Picture(0x1605U, 480U);
        LCD_Disappear_Number(0x9530U);
    }
}

void Infusiondisplay(uint8_t activation_flag, uint16_t inject_data)
{
    if (activation_flag != 0U)
    {
        LCD_Show_Picture(0x1415U, 383U);
        LCD_Show_Picture(0x1417U, 499U);
        LCD_Show_Picture(0x1418U, 501U);
        LCD_Show_Picture(0x1419U, 481U);
        LCD_Show_Number(0x9550U, 0x3550U);
        LCD_Show_4byte_Number(0x3550U, inject_data);
        LCD_Show_Picture(0x1416U, (uint16_t)(387U + Screen_PumpGear_FromValue(inject_data, 30U, 300U)));
    }
    else
    {
        LCD_Show_Picture(0x1415U, 382U);
        LCD_Show_Picture(0x1417U, 498U);
        LCD_Show_Picture(0x1418U, 500U);
        LCD_Show_Picture(0x1419U, 480U);
        LCD_Show_Picture(0x1420U, 384U);
        LCD_Show_Picture(0x1416U, 387U);
        LCD_Disappear_Number(0x9550U);
    }
}

void draindisplay(uint8_t drain_flag)
{
    LCD_Show_Picture(0x1606U, (uint16_t)(484U + (drain_flag > 2U ? 2U : drain_flag)));
}

void Irrndisplay(uint8_t Irrnd_flag)
{
    LCD_Show_Picture(0x1420U, (uint16_t)(384U + (Irrnd_flag > 2U ? 2U : Irrnd_flag)));
}

void clockwisedisplay(uint8_t clockwise_flag)
{
    LCD_Show_Picture(0x1408U, (uint16_t)(360U + (clockwise_flag > 2U ? 2U : clockwise_flag)));
}

void anticlockwisedisplay(uint8_t anticlockwise_flag)
{
    LCD_Show_Picture(0x1410U, (uint16_t)(363U + (anticlockwise_flag > 2U ? 2U : anticlockwise_flag)));
}

void OSCdiplay(uint8_t OSC_flag)
{
    LCD_Show_Picture(0x1409U, (uint16_t)(366U + (OSC_flag > 2U ? 2U : OSC_flag)));
}

void Tooldisplay(uint8_t tool_data)
{
    if (tool_data == 0U)
    {
        LCD_Disappear_Picture(0x1406U);
    }
    else
    {
        LCD_Show_Picture(0x1406U, (uint16_t)(509U + tool_data));
    }
}

void OPENpostionDisplay(uint8_t openpostion_flag)
{
    if (openpostion_flag != 0U)
    {
        LCD_Show_Picture(0x1405U, 400U);
    }
    else
    {
        LCD_Disappear_Picture(0x1405U);
    }
}

void PAOORMODisplay(uint8_t PAOORMO_VALUE)
{
    if (PAOORMO_VALUE == 0U)
    {
        LCD_Disappear_Picture(0x1403U);
        LCD_Disappear_Picture(0x1404U);
        OPENpostionDisplay(0U);
    }
    else if (PAOORMO_VALUE == 1U)
    {
        OPENpostionDisplay(0U);
        LCD_Show_Picture(0x1404U, 421U);
        LCD_Show_Picture(0x1403U, 422U);
    }
    else
    {
        OPENpostionDisplay(1U);
        LCD_Show_Picture(0x1404U, 420U);
        LCD_Show_Picture(0x1403U, 423U);
    }
}

void AutomaticDisplay(uint8_t activation_flag)
{
    KeyBeep_flag = 1U;
    LCD_Show_Picture(0x1440U, activation_flag ? 424U : 425U);
}

void AutomaticAxtion(uint8_t paodao_flag)
{
    if (paodao_flag != 0U)
    {
        OPENpostionDisplay(1U);
        clockwisedisplay(1U);
        anticlockwisedisplay(1U);
        OSCdiplay(2U);
    }
    else
    {
        OPENpostionDisplay(0U);
        clockwisedisplay(2U);
        anticlockwisedisplay(1U);
        OSCdiplay(0U);
    }
}

void specidisplay(uint8_t speci_flag, uint16_t Length, uint8_t Diameter, uint8_t Angle)
{
    if (speci_flag != 0U)
    {
        LCD_Show_2byte_Number(0x8008U, 34U);
        LCD_IntegratedCutterData_Update(0x4200U, Length, Diameter, Angle);
    }
    else
    {
        LCD_Show_2byte_Number(0x8008U, 0U);
    }
}

void ALARMdisplay(void)
{
    Screen_TipInfo_Update(WorkMessage.alarm_flag ? WorkMessage.alarm_value : 0U);
}

uint8_t APump(uint8_t pump_data)
{
    uint32_t temp = ((uint32_t)pump_data * 2U) + 2U;
    return (uint8_t)(temp > 255U ? 255U : temp);
}

void InsertHandControl(uint8_t key_value)
{
    (void)key_value;
}

void ScreenKeyTask_Init(void) {}
void FootKeyTask_Init(void) {}
void BeepControlTask_Init(void) {}
void PUMPBTask_Init(void) {}
void SscDisplayInit(void) {}
void SscDisplayManage(void) {}

void Screen_Password_Input(uint8_t Position)
{
    LCD_Show_Picture(0x1200U, (uint16_t)(330U + Position));
}

void Info_A(uint8_t Info1)
{
    s_info_a = Info1;
    Screen_InformationBarImage_Update(s_info_a, s_info_b, s_info_hz, s_info_extra);
}

void Info_B(uint8_t Info1)
{
    s_info_b = Info1;
    Screen_InformationBarImage_Update(s_info_a, s_info_b, s_info_hz, s_info_extra);
}

void Info_HZ(uint8_t Info3)
{
    s_info_hz = Info3;
    Screen_InformationBarImage_Update(s_info_a, s_info_b, s_info_hz, s_info_extra);
}

void Screen_InformationBarImage_Update(uint8_t Info1, uint8_t Info2, uint8_t Info3, uint8_t Info4)
{
    s_info_a = Info1;
    s_info_b = Info2;
    s_info_hz = Info3;
    s_info_extra = Info4;
    LCD_Show_Picture(0x1600U, (uint16_t)(430U + Info1));
    LCD_Show_Picture(0x1601U, (uint16_t)(440U + Info2));
    LCD_Show_Picture(0x1602U, (uint16_t)(450U + Info3));
    LCD_Show_Picture(0x1603U, (uint16_t)(460U + Info4));
}

void Screen_IntegratedCutterPic_Update(uint8_t IntegratedCutter)
{
    if (IntegratedCutter == 0U)
    {
        LCD_Disappear_Picture(0x1604U);
        LCD_Disappear_Picture(0x1605U);
    }
    else
    {
        LCD_Show_Picture(0x1604U, 380U);
        LCD_Show_Picture(0x1605U, (uint16_t)(380U + IntegratedCutter));
    }
}

void Screen_SeparatingCutterPic_Update(uint8_t SeparatingCutter, uint8_t WFFlag)
{
    if (SeparatingCutter == 0U)
    {
        LCD_Disappear_Picture(0x1604U);
        LCD_Disappear_Picture(0x1605U);
        LCD_Disappear_Picture(0x1606U);
        LCD_Disappear_Picture(0x4200U);
        return;
    }

    LCD_Show_Picture(0x1604U, (SeparatingCutter < 4U) ? 381U : 382U);
    LCD_Show_Picture(0x1605U, (uint16_t)(382U + SeparatingCutter));
    if ((WFFlag != 0U) || (SeparatingCutter == 4U))
    {
        LCD_Show_Picture(0x1606U, 400U);
    }
    else
    {
        LCD_Disappear_Picture(0x1606U);
    }
}

uint8_t *Screen_HandleConnectState_Update(uint8_t *HandleConn, uint8_t *HandleType)
{
    uint8_t left_connected = 0U;
    uint8_t right_connected = 0U;

    if ((HandleConn == NULL) || (HandleType == NULL))
    {
        return s_handle_order;
    }

    left_connected = (HandleConn[1] > 0U) || (HandleConn[2] > 0U);
    right_connected = (HandleConn[3] > 0U) || (HandleConn[4] > 0U);
    s_handle_order[0] = left_connected ? 2U : 0U;
    s_handle_order[1] = right_connected ? 4U : 0U;
    s_handle_order[4] = left_connected ? 1U : (right_connected ? 2U : 0U);

    Screen_ShowSimpleHandle(0x1500U, left_connected, s_handle_order[4] == 1U);
    Screen_ShowSimpleHandle(0x1501U, right_connected, s_handle_order[4] == 2U);
    return s_handle_order;
}

void Screen_FootPedalConnectState_Update(uint8_t Type)
{
    static const uint16_t pic_table[] = {0U, 370U, 374U, 377U, 372U, 371U};

    if (Type == 0U)
    {
        LCD_Disappear_Picture(0x1603U);
    }
    else if (Type < (sizeof(pic_table) / sizeof(pic_table[0])))
    {
        LCD_Show_Picture(0x1603U, pic_table[Type]);
    }
}

void Screen_ElectricalMachineryDirectionState_Update2(uint8_t Dir1)
{
    static const uint16_t pic_table[] = {360U, 361U, 362U, 363U, 364U, 365U, 366U};
    uint8_t index = (Dir1 < (sizeof(pic_table) / sizeof(pic_table[0]))) ? Dir1 : 0U;
    LCD_Show_Picture(0x1602U, pic_table[index]);
}

void Screen_ElectricalMachineryDirectionState_Update(uint8_t Dir1, uint8_t Dir2, uint8_t Dir3)
{
    if (Dir2 != 0U)
    {
        Screen_ElectricalMachineryDirectionState_Update2(2U);
    }
    else if (Dir3 != 0U)
    {
        Screen_ElectricalMachineryDirectionState_Update2(3U);
    }
    else
    {
        Screen_ElectricalMachineryDirectionState_Update2(Dir1);
    }
}

void Screen_TipInfo_Update(uint8_t TipID)
{
    static const uint16_t pic_table[] = {
        0U, 407U, 403U, 408U, 404U, 406U, 410U, 405U, 409U, 411U, 410U, 411U, 412U, 413U
    };

    if (TipID == 0U)
    {
        LCD_Disappear_Picture(0x1607U);
    }
    else if (TipID < (sizeof(pic_table) / sizeof(pic_table[0])))
    {
        LCD_Show_Picture(0x1607U, pic_table[TipID]);
    }
}

void Screen_WindowSwitch_Update(int8_t Num1, uint8_t Num2)
{
    (void)Num1;
    LCD_Disappear_Picture(0x1410U);
    LCD_Disappear_Picture(0x1411U);
    LCD_Disappear_Picture(0x1412U);
    LCD_Disappear_Picture(0x1414U);
    LCD_Disappear_Picture(0x1415U);

    switch (Num2)
    {
        case 0U: LCD_Show_Picture(0x1410U, 310U); break;
        case 1U: LCD_Show_Picture(0x1411U, 312U); break;
        case 2U: LCD_Show_Picture(0x1413U, 313U); break;
        case 3U: LCD_Show_Picture(0x1415U, 315U); break;
        default: break;
    }
}
