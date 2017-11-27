#ifndef __PID_H
#define __PID_H
#include "config_param.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * PID��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    33.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   33.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     167.0

/*���ٶȻ������޷�*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		20.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	20.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		360.0

#define DEFAULT_PID_INTEGRATION_LIMIT  		5000.0

typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float iLimitLow;    //< integral limit
	float dt;           //< delta-time dt
} PidObject;

/*pid�ṹ���ʼ��*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid�����޷�����*/
void pidSetDesired(PidObject* pid, const float desired);	/*pid��������ֵ*/
float pidUpdate(PidObject* pid, const float error);			/*pid����*/
float pidGetDesired(PidObject* pid);	/*pid��ȡ����ֵ*/
bool pidIsActive(PidObject* pid);		/*pid״̬*/
void pidReset(PidObject* pid);			/*pid�ṹ�帴λ*/
void pidSetError(PidObject* pid, const float error);/*pidƫ������*/
void pidSetKp(PidObject* pid, const float kp);		/*pid Kp����*/
void pidSetKi(PidObject* pid, const float ki);		/*pid Ki����*/
void pidSetKd(PidObject* pid, const float kd);		/*pid Kd����*/
void pidSetDt(PidObject* pid, const float dt);		/*pid dt����*/

#endif /* __PID_H */
