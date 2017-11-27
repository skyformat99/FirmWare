#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���ò�����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
												
typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;	
} pidParamPos_t;

typedef struct	
{
	u8 version;				/*����汾��*/
	pidParam_t pidAngle;	/*�Ƕ�PID*/	
	pidParam_t pidRate;		/*���ٶ�PID*/	
	pidParamPos_t pidPos;	/*λ��PID*/
	u16 thrustBase;			/*���Ż���ֵ*/
	u8 cksum;				/*У��*/
} configParam_t;


extern configParam_t configParam;

void configParamInit(void);	/*�������ó�ʼ��*/
void configParamTask(void* param);	/*������������*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void resetConfigParamPID(void);

#endif /*__CONFIG_PARAM_H */

