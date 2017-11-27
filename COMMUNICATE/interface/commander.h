#ifndef __COMMANDER_H
#define __COMMANDER_H
#include "atkp.h"
#include "config.h"
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��ȡң��������������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

typedef struct
{
	u8 altHoldMode	: 1;	/*bit0 ģʽ 1=���� 0=�ֶ�*/
	u8 keyFlight 	: 1;	/*bit1 һ�����*/
	u8 keyLand 		: 1;	/*bit2 һ������*/
	u8 emerStop 	: 1;	/*bit3 ����ͣ��*/
	u8 flightMode 	: 1;	/*bit4 ����ģʽ 1=��ͷ 0=��ͷ*/
	u8 reserved		: 3;	/*bit5~7 ����*/
}commanderBits_t;

/*�������ݽṹ��*/
typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	u16 thrust;
} ctrlVal_t;

/*���ݻ���ṹ��*/
typedef struct
{
	ctrlVal_t  tarVal[2];
	bool activeSide;
	u32 timestamp; 		/* FreeRTOS ʱ�ӽ���*/
} ctrlValCache_t;

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*Xģʽ*/
	CAREFREE  = 1, /*��ͷģʽ*/
} YawModeType;

typedef enum
{
	ATK_REMOTER = 0,
	WIFI		= 1,
}ctrlSrc_e;
	
void commanderInit(void);
bool commanderTest(void);
void flightCtrldataCache(ctrlSrc_e ctrlSrc, ctrlVal_t pk);
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);
void flyerAutoLand(setpoint_t *setpoint,const state_t *state);

void getCommanderTrim(float* pitch, float* roll);

void setCommanderAltholdMode(bool set);
bool getCommanderAltholdMode(void);

void setCommanderKeyFlight(bool set);
bool getCommanderKeyFlight(void);

void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);

void setCommanderFlightmode(bool set);
void setCommanderEmerStop(bool set);

#endif /* __COMMANDER_H */
