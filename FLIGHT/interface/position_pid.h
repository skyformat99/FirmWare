#ifndef __POSITION_PID_H
#define __POSITION_PID_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * λ��PID���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define POS_UPDATE_RATE 		RATE_250_HZ
#define POS_UPDATE_DT 			(1.0f / POS_UPDATE_RATE)

void positionControlInit(void);
void positionResetAllPID(void);
void altholdPID(float *thrust, const state_t *state, const setpoint_t *setpoint);
void getPositionPIDZ(float* kp, float* ki, float* kd);
void setPositionPIDZ(float kp, float ki, float kd);
	
#endif /* __POSITION_PID_H */
