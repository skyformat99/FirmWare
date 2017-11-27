#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_control.h"
#include "flip.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �������ȿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;
static bool isCalibPass;
static setpoint_t 	setpoint;	/*����Ŀ��״̬*/
static sensorData_t sensorData;	/*����������*/
static state_t 		state;		/*������̬*/
static control_t 	control;	/*������Ʋ���*/

void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;

	stateControlInit();		/*��̬PID��ʼ��*/
	powerControlInit();		/*�����ʼ��*/

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerControlTest();

	return pass;
}


void stabilizerTask(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	
	ledseqRun(SYS_LED, seq_alive);
	
	while(!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
	}
	isCalibPass = true;
	vTaskDelay(1000);	/*��ʱ�ȴ��������ȶ�*/
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms������ʱ*/

		sensorsAcquire(&sensorData, tick);					/*��ȡ6�����ѹ����*/
		stateEstimator(&state, &sensorData, tick);			/*��Ԫ�� ŷ���Ǽ���*/
		
		commanderGetSetpoint(&setpoint, &state);			/*Ŀ�����ݺͷ���ģʽ�趨*/	
		flyerFlipCheck(&setpoint, &control, &state, tick);	/*�������*/
		anomalDetec(&sensorData, &state, &control);			/*�쳣���*/
		stateControl(&control, &sensorData, &state, &setpoint, tick);/*PID����*/		
		powerControl(&control);								/*���Ƶ�����*/

		tick++;
	}
}

void getAttitudeData(attitude_t* get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
//	*get = state.attitude;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

float getPositionZ(void)
{
	return state.position.z;
}

bool getIsCalibPass(void)
{
	return isCalibPass;
}

void getState(state_t* get)
{
	*get = state;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}
