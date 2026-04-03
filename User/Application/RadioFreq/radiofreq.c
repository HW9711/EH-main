//radiofreq.c

#include "radiofreq.h"
#include "data.h"

#include "uart3.h"
#include "delay.h"

#define RF_BUFF_LENGTH            254
#define Frame_header              0xbb
#define Frame_header_Second       0x01
#define Frame_tail                0x7e

#define Read_Flag                 0x39
#define Write_Flag                0x49
#define Read_OR_Write_File_Flag   0xff
#define READ_DATA_BUFF_LENGTH     16

PIDREG_T     pi_spd = PIDREG_T_DEFAULTS;
PIDREG_T     pi_ICurr = PIDREG_T_DEFAULTS;

unsigned char hop_ch[]={0XBB ,0X00 ,0XAD ,0X00 ,0X01 ,0XFF, 0XAD ,0X7E };//使用跳屏
unsigned char hop_return[]={0xBB ,0x01 ,0xAD ,0x00 ,0x01 ,0x00 ,0xAF ,0x7E};//跳屏返回码
unsigned char stop_hop_ch[]={0XBB ,0X00 ,0XAD ,0X00, 0X01, 0x00 ,0xAE ,0x7E};//停止跳屏

unsigned char  pa_gain20[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X0A, 0X28, 0Xea, 0X7E};//发射功率
unsigned char  pa_gain10[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X03, 0Xe8, 0Xa3, 0X7E};//发射功率
unsigned char  pa_gain3[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X01, 0X2C, 0XE5, 0X7E}; //发射功率
unsigned char  pa_gain0[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X00, 0X00, 0XB8, 0X7E}; //发射功率
unsigned char  pa_gain5[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X01, 0XF4, 0XAD, 0X7E}; //发射功率

//unsigned char single_reading[7]= {0XBB,0X00,0X22,0x00,0X00,0x22,0x7E} ;//单次读取
unsigned char stop_distinguish[]={0XBB, 0X00, 0X28, 0X00, 0X00, 0X28, 0X7E};//停止多标签读取
unsigned char  more_distinguish[]={0XBB, 0X00, 0X27, 0X00, 0X03, 0X22, 0XFF, 0XFF, 0X4A, 0X7E};//多标签读取


unsigned char region_set_europe[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x03 ,0x0B ,0x7E};//欧洲频段865.1-867.9M
unsigned char region_set_us[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x02 ,0x0A ,0x7E};//美国频段902.25-927.75M
unsigned char region_set_CHAIN[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x01 ,0x09 ,0x7E};//中国1-920.125-924.875M
unsigned char region_set_CHAINS[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x04 ,0x0C ,0x7E};//中国2-840.125-844.875M
unsigned char region_set_K[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x06 ,0x0E ,0x7E};//韩国-917.1-923.3M

unsigned char channel_number[]={0xBB ,0x00 ,0xAB ,0x00 ,0x01 ,0x00 ,0xAC ,0x7E };//各个区域的第一信道

unsigned char write_data_return_error[]={0xBB ,0x01, 0xFF,0x00 ,0x01 ,0x10 ,0x11 ,0x7e};//返回 单次读取失败回码

unsigned char read_second_data_return_error[]={0xBB ,0x01 ,0xFF ,0x00 ,0x01 ,0x09 ,0x0A ,0x7E};//读取user失败回码

unsigned char read_frist_data_return_error[]={0xBB ,0x01 ,0xFF ,0x00 ,0x01 ,0x15 ,0x16 ,0x7E};//读取epc失败回码

unsigned char set_gain_return[]={0xBB ,0x01 ,0xB6 ,0x00 ,0x01 ,0x00 ,0xB8 ,0x7E};//增益

unsigned char set_region_return[]={0xBB ,0x01 ,0x07 ,0x00 ,0x01 ,0x00 ,0x09 ,0x7E};//区域

unsigned char ReadTID[16]={0xBB , 0x00 , 0x39 , 0x00 , 0x09 , 0x00 , 0x00 , 0x00 , 0x00 , 0x02 , 0x00 , 0x00 , 0x00 , 0x06 , 0x4A , 0x7E };//TID读取BB 00 39 00 09 00 00 00 00 02 00 00 00 06 4A 7E

unsigned char EPC_NO_MASK1[16]={0xBB , 0x00 , 0x39 , 0x00 , 0x09 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x08 , 0x4b , 0x7E};//无掩码读取EPC

unsigned char Check_need_MASK2[16]={0xBB , 0x00 , 0x39 , 0x00 , 0x09 , 0x00 , 0x00 , 0x00 , 0x00 , 0x02 , 0x00 , 0x00 , 0x00 , 0x06 , 0x4a , 0x7E};//无掩码读取tid

//unsigned char NO_MASK3_READ_USER[16]={0xBB,0x00,0x39,0x00,0x09,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x08,0x4d,0x7E};//无掩码读取USER
//unsigned char NO_MASK3_READ_USER[16]={0xBB , 0x00 , 0x39 , 0x00 , 0x09 , 0x00 , 0x00 , 0x00 , 0x00 , 0x03 , 0x00 , 0x00 , 0x00 , 0x08 , 0x4d , 0x7E};//无掩码读取USER

unsigned char NO_MASK3_WRITE_USER[32]={0xBB,0x00,0x49,0x00,0x19,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x08,
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x7E};

unsigned char no_mask_model[14]={0xBB , 0x00 , 0x0C , 0x00 , 0x07 , 0x23 , 0x00 , 0x00 , 0x00 , 0x00 , 0x60 , 0x00 , 0x96 , 0x7E};//暂时用不上

unsigned char  BaudRate9600[9]={ 0xBB  , 0x00  , 0x11  , 0x00  , 0x02  , 0x00  , 0x60  , 0x73  , 0x7E};//


//extern u32 Motor_Set_Speed;
//extern u8 Motor_Number;
//速度PID

#define  Speed_Kp2    0.02 //0.1
#define  Speed_Ki2    0.01// 0.01
#define  Speed_Kc2    0.01/// 0.01

#define  Speed_Kp1    0.10 //0.1
#define  Speed_Ki1    0.01// 0.01
#define  Speed_Kc1    0.01/// 0.01

/**************PID参数初始化******************/
void PID_init2(void)
{
  pi_spd.Kp = Speed_Kp2;
  pi_spd.Ki = Speed_Ki2;
  pi_spd.Kc = Speed_Kc2;
  pi_spd.OutMax = 3700;
  pi_spd.OutMin =0;
  pi_spd.Ref = 0;
  pi_spd.Err = 0;
  pi_spd.Up = 0;
  pi_spd.Up1 = 0;
  pi_spd.Ui = 0;
  pi_spd.Ud = 0;
  pi_spd.Out = 0;
}

void PID_init1(void)
{
  pi_spd.Kp = Speed_Kp1;
  pi_spd.Ki = Speed_Ki1;
  pi_spd.Kc = Speed_Kc1;
  pi_spd.OutMax = 3700;
  pi_spd.OutMin =0;
  pi_spd.Ref = 0;
  pi_spd.Err = 0;
  pi_spd.Up = 0;
  pi_spd.Up1 = 0;
  pi_spd.Ui = 0;
  pi_spd.Ud = 0;
  pi_spd.Out = 0;
}

//============================================================================
// 函数名称: RadioFreq_Init()
// 功能描述: 射频初始化
// 输　  入: 功率length，频段，频段length
// 输    出:
// 函数说明:
//============================================================================
void RadioFreq_Init(void)
{
  Uart3_SendPacket(pa_gain0, 9);  //功率设置
  Delay_ms(50);

  Uart3_SendPacket(region_set_CHAIN, 8);  //区域设置
  Delay_ms(50);

  Uart3_SendPacket(hop_ch, 8);//跳频hop_ch，取消跳频stop_hop_ch
  Delay_ms(50);
}

//============================================================================
// 函数名称: RadioFreq_CRC()
// 功能描述: 射频数据CRC校验
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
uint8_t RadioFreq_CRC(uint8_t *data, uint8_t length, uint8_t startposition)
{
  int total = 0;
  uint16_t i = 0;

  for(i = 0; i < length; i++)
  {
	  total += data[startposition + i];
  }

  if (data[length + startposition] == ((uint8_t)(total) & 0xff))
  {
    return 1;
  }

  return 0;
}

//============================================================================
// 函数名称: RadioFreq_Analysiss()
// 功能描述: 射频数据CRC校验
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
uint8_t RadioFreq_Analysiss(uint8_t * uart1_rf_buff, uint8_t * datbuf)
{
  uint16_t i = 0;
  uint8_t crc_rsult = 0;

  //记得用上volition

  uint8_t get_data_falg = 0;
  uint8_t start_position = 0;
  uint8_t head_falg = 0;

  for (i = 0; i < RF_BUFF_LENGTH; i++)
  {
	  //1.找帧头并确定起始下标
    if (!head_falg)
	  {
	    //未找到帧头...
	    if (RF_BUFF_LENGTH <= i)
      {
		    break;
	    }

	    //帧头 0xbb 0x01 0x39
	    if ((uart1_rf_buff[i] == Frame_header) && (uart1_rf_buff[i + 1] == Frame_header_Second) && (uart1_rf_buff[i + 2] == Read_Flag))
	    {
	      start_position = i;
	      head_falg = 1;
	    }
    }
	  //2.已找到帧头并确定了起始下标，
	  else
	  {
	    //0x7e
	    if (uart1_rf_buff[i] == Frame_tail)
	    {
		    if (uart1_rf_buff[start_position + 4] == (i - start_position - 6))
		    {
		      crc_rsult =	RadioFreq_CRC(uart1_rf_buff, uart1_rf_buff[start_position + 4] + 4, start_position + 1);
		      if (crc_rsult)
		      {
			      //正确
			      //	delay_ms(100);
			      switch (uart1_rf_buff[start_position + 2])
			      {
			        case Write_Flag : get_data_falg = 1; break;
			        case Read_Flag :
			        {
				        get_data_falg = 2;
				        start_position = uart1_rf_buff[start_position + 5] + 6 + start_position;
				        for (i = 0; i < READ_DATA_BUFF_LENGTH; i++)
				        {
				          datbuf[i] = uart1_rf_buff[start_position + i];
				        }
			        }
			        break;
			        case Read_OR_Write_File_Flag : get_data_falg = 3; break;
			        default : get_data_falg = 0; break;
			      }
			      break;
		      }
		      else
		      {
			      head_falg = 0;
			      i = start_position + 3;
		      }
		    }
		    else if (uart1_rf_buff[start_position + 4] < (i - start_position - 6))  //小于这个
		    {
		      i = start_position + 3;
		      head_falg = 0;
		    }
	    }
	  }
  }

  return get_data_falg;
}






























