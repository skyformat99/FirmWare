#include "string.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "state_control.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "config_param.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ������̬���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;


void stateControlInit(void)
{
	attitudeControlInit();	/*��ʼ����̬PID*/	
	positionControlInit();	/*��ʼ��λ��PID*/
}

bool stateControlTest(void)
{
	bool pass = true;
	pass &= attitudeControlTest();
	return pass;
} 

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick)
{
	static u16 cnt = 0;
	
	if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick))		/* 250Hz */ 
	{
		altholdPID(&actualThrust, state, setpoint);		/* ����PID(�ٶ�ģʽ)*/
	}

	if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick))	/* 500Hz */  
	{
		if(setpoint->isAltHold == false) 
		{
			actualThrust = setpoint->thrust;
		}
		if(control->flipDir == CENTER)
		{
			attitudeDesired.yaw -= setpoint->attitude.yaw/ATTITUDE_UPDATE_RATE;	/*����YAW ����ģʽ*/
			while(attitudeDesired.yaw > 180.0f) 
				attitudeDesired.yaw -= 360.0f;
			while(attitudeDesired.yaw < -180.0f) 
				attitudeDesired.yaw += 360.0f;
		}
		attitudeDesired.roll = setpoint->attitude.roll;		/*����ROLL �Ƕ�(����)ģʽ*/
		attitudeDesired.pitch = setpoint->attitude.pitch;	/*����PITCH �Ƕ�(����)ģʽ*/
		
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);	/* �Ƕ�(�⻷)PID */
		
		if(control->flipDir != CENTER)	/*�շ���ʱ��ֻʹ���ڻ�PID*/
		{
			rateDesired.pitch = setpoint->attitude.pitch;
			rateDesired.roll = setpoint->attitude.roll;
		}
		attitudeRatePID(&sensors->gyro, &rateDesired, control);				/* ���ٶ�(�ڻ�)PID */

	#ifdef ENABLE_PID_TUNING	/* ʹ��PID���� yawֵ������ */
		control->yaw = 0;	
	#endif
	}

	control->thrust = (u16)actualThrust;	

	if (control->thrust == 0)
	{			
		control->thrust = 0;
		control->roll = 0;
		control->pitch = 0;
		control->yaw = 0;
		
		attitudeResetAllPID();	/*��λ��̬PID*/	
		positionResetAllPID();	/*��λλ��PID*/
		attitudeDesired.yaw = state->attitude.yaw;		/*��λ���������yawֵ*/
		
		float velZ = state->velocity.z;
		if(velZ<0.001f && velZ>-0.001f && cnt++>1000)	/*������ɺ󱣴����*/
		{
			cnt = 0;
			configParamGiveSemaphore();
		}
	}

}


