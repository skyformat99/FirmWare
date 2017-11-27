#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H
#include <stdbool.h>
#include "commander.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��̬PID���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define ATTITUDE_UPDATE_RATE 	RATE_500_HZ
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)

void attitudeControlInit(void);
bool attitudeControlTest(void);

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output);	/* ���ٶȻ�PID */
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);	/* �ǶȻ�PID */
void attitudeResetAllPID(void);		/*��λPID*/
void attitudePIDwriteToConfigParam(void);

#endif /* __ATTITUDE_PID_H */
