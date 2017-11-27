#include <math.h>
#include "sensfusion6.h"
#include "state_estimator.h"
#include "attitude_pid.h"
#include "position_pid.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��̬�������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define G 9.78f;

static selfState_t selfState = 
{
	.estimatedZ = 0.0f,
	.velocityZ = 0.0f,
	.estAlpha = 0.97f,
	.velocityFactor = 1.0f,
	.vAccDeadband = 0.04f,
	.velZAlpha = 0.995f,
	.velocityX = 0.0f,
	.velocityY = 0.0f,
};

float deadband(float value, const float threshold)
{
	if (fabs(value) < threshold)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= threshold;
	}
	else if (value < 0)
	{
		value += threshold;
	}
	return value;
}

static void positionEstimate(state_t* state, float asl, float dt) 
{
	float filteredZ;
	
	if(selfState.estimatedZ == 0.0f)
		filteredZ = asl;
	else
		filteredZ = selfState.estAlpha * selfState.estimatedZ + (1.0f - selfState.estAlpha) * asl;
	
	selfState.estimatedZ = filteredZ + selfState.velocityFactor * selfState.velocityZ * dt;

	state->position.x = 0.0f;
	state->position.y = 0.0f;
	state->position.z = selfState.estimatedZ;
}

static void stateEstimateVelocityZ(state_t *state, float dt) 
{	
	selfState.velocityZ += deadband(state->acc.z, selfState.vAccDeadband) * dt * G;	/* Vt=V0+ at */
	selfState.velocityZ *= selfState.velZAlpha;
	state->velocity.z = selfState.velocityZ;
}

void stateEstimator(state_t *state, const sensorData_t *sensorData, const u32 tick)
{
	if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) /* 500Hz 2ms update  */
	{	  
		imuUpdate(sensorData->acc, sensorData->gyro, state, ATTITUDE_UPDATE_DT);
		stateEstimateVelocityZ(state, ATTITUDE_UPDATE_DT);
	}
	
	if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) /*250Hz 4ms update */
	{  		
		positionEstimate(state, sensorData->baro.asl, POS_UPDATE_DT);		
	}
}

