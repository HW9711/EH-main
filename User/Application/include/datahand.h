#include <stdint.h>
#include "stm32f4xx_hal.h"


#define TMBB_ONLINE 1
#define TMBA_ONLINE 2
#define EMBA_ONLINE 3
#define EMBB_ONLINE 4
#define PXBA_ONLINE 5
#define PXBB_ONLINE 6
#define MX_YIM_ONLINE 7  //磨削 一体磨
#define MX_YIP_ONLINE 8  //磨削 一体刨
#define PX_YIM_ONLINE 9  //刨削 一体磨
#define PX_YIP_ONLINE 10 //刨削 一体刨
#define JMB_ONLINE    11 //
#define MX_YIM16_ONLINE    12 //



#define FootControl    		 1   //脚控
#define HandControl    		 2   //手控
#define TouchControl    	 3   //触控

#define start_on           1   //开
#define stop_close         0   //关

#define ModeSlect_M        1  	//磨头
#define ModeSlect_P        1  	//刨刀

#define HandleChannel_A    1    //a通道
#define HandleChannel_B    1    //b通道
#define HandleChannel_NO   1    //b通道

#define Handle_connect     1    //手柄连接
#define Handle_select      2    //手柄选中
#define Handle_on       	 0    //无手柄


typedef struct
{
	//	volatile uint8_t Handel_icon;
	//	volatile uint8_t speed_icon;							//速度栏图标
	//	volatile uint8_t pump_icon;								//泵流量图标
	//	volatile uint8_t pump_button_icon; 				//泵操作按钮图标
	//	volatile uint8_t sports_icon;							//运动图标
	//	volatile uint8_t frequency_icon;					//频率图标
	//	volatile uint8_t manualRecognition_icon; 	//手动识别是否开启识别flag
	//	volatile uint8_t mode_M_P_Slect_icon; 		//磨头还是刨刀
	//	volatile uint8_t control_model_icon;			//控制模式图片
	volatile uint16_t set_speed_data;							//速度
	volatile uint16_t min_speed_data;
	volatile uint16_t max_speed_data;
	
	volatile uint8_t set_pump_data;//注水泵
	volatile uint8_t min_pump_data;
	volatile uint8_t max_pump_data;
	
	volatile uint8_t set_freq_data;//频率
	volatile uint8_t min_freq_data;
	volatile uint8_t max_freq_data;
	
}DataShow;

typedef struct
{
	volatile uint8_t tmba_pump;
	volatile uint8_t tmbb_pump;
	volatile uint8_t emba_pump;
	volatile uint8_t embb_pump;
	volatile uint8_t pxbx_pump;
	volatile uint8_t mx_pump;
	volatile uint8_t px_pump;
	volatile uint8_t tmba_speed;
	volatile uint8_t tmbb_speed;
	volatile uint8_t emba_speed;
	volatile uint8_t embb_speed;
	volatile uint8_t mx_ytp_speed;
	volatile uint8_t mx_ytm_speed;
	volatile uint8_t px_ytp_speed;
	volatile uint8_t px_ytm_speed;
	volatile uint8_t px_p_speed;
	volatile uint8_t px_m_speed;
	
	DataShow A_DataShow;
	DataShow B_DataShow;
	DataShow A_MT_DataShow;//磨头
	DataShow A_PD_DataShow;//刨刀
	DataShow B_MT_DataShow;//磨头
	DataShow B_PD_DataShow;//刨刀
}
UIDataShow;
extern UIDataShow UIDataShow_s; 

typedef struct 
{
	volatile uint8_t HandleChannel_state;//0表示无连接，1表示选中，2表示连接
	volatile uint8_t OnlineModel;        //手柄在线型号，
	volatile uint8_t SportModel;         //运动模式，正，反，往复
	volatile uint8_t ControlModel;       //控制模式，脚控，手控，触控

	volatile uint8_t manualRecognition_flag; //手动模式开启
	volatile uint8_t mode_M_P_Slect_mode;    //刨刀或者磨头
}
ControlState;

typedef struct 
{
	ControlState A_ControlState;
	ControlState B_ControlState;
  volatile uint8_t HandleselectionChannel;//手柄选中当前通道,0无通道，1A通道，2B通道
	volatile uint8_t Handel_switch_flag;//手柄切换信号
	volatile uint8_t IrrigateSwitch_flag; //灌注启动flag
	volatile uint8_t InjectionSwitch_flag;//注水启动flag
	ControlState A_MT_ControlState;
	ControlState A_PD_ControlState;
	
	ControlState B_MT_ControlState;
	ControlState B_PD_ControlState;
}
UIControlState;
extern UIControlState UIControlState_s;

void SscUIDataInit(void);
void SscDataUpdate(void);
