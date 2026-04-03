//radiofreq.h

#ifndef __RADIOFREQ_H
#define __RADIOFREQ_H

#include <stdint.h>

//定义PID结构体
typedef struct
{
				float  Ref;   		// Input: Reference input
				float  Fdb;   		// Input: Feedback input
				float  Err;			  // Variable: Error
				float  Kp;			  // Parameter: Proportional gain
				float  Up;			  // Variable: Proportional output
				float  Ui;			  // Variable: Integral output
				float  Ud;			  // Variable: Derivative output
				float  OutPreSat;	// Variable: Pre-saturated output
				float  OutMax;		// Parameter: Maximum output
				float  OutMin;		// Parameter: Minimum output
				float  Out;   		// Output: PID output
				float  SatErr;		// Variable: Saturated difference
				float  Ki;			  // Parameter: Integral gain
				float  Kc;			  // Parameter: Integral correction gain
				float  Kd; 			  // Parameter: Derivative gain
				float  Up1;			  // History: Previous proportional output
				float  Ui_1;
				float  OutF;
} PIDREG_T ,*p_PIDREG_T ;;

#define PIDREG_T_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数

#define   _IQmpy(A,B)         ((A) * (B))
#define FirstOrder_LPF_Cacl(Xn, Yn_1, a)\
																					Yn_1 = (1-a)*Yn_1 + a*Xn; //Xn:in;Yn:out;a:系数
#define UP16LIMIT(var,max,min) {(var) = (var)>(max)?(max):(var) ;\
																					(var) = (var)<(min)?(min):(var) ;\
																					}
#define PID_CALC(v)	\
											v.Err = v.Ref - v.Fdb; \
											v.Up= _IQmpy(v.Kp,v.Err);\
											v.Ui= v.Ui + _IQmpy(v.Ki,v.Up);\
											UP16LIMIT(v.Ui,v.OutMax,v.OutMin);\
											v.Ud = v.Kd * (v.Up - v.Up1);\
											v.Out = v.Up + v.Ui + v.Ud;\
											UP16LIMIT(v.Out,v.OutMax,v.OutMin);\
											v.Up1 = v.Up;


void PID_init1(void);
void PID_init2(void);

//============================================================================
// 函数名称: RadioFreq_Init()
// 功能描述: 射频初始化
// 输　  入: 功率length，频段，频段length
// 输    出:
// 函数说明:
//============================================================================
void RadioFreq_Init(void);

//============================================================================
// 函数名称: RadioFreq_CRC()
// 功能描述: 射频数据CRC校验
// 输　  入:
// 输    出:
// 函数说明:
//============================================================================
uint8_t RadioFreq_Analysiss(uint8_t * uart1_rf_buff, uint8_t * datbuf);


#endif  //__RADIOFREQ_H



