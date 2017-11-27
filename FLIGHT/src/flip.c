#include <math.h>
#include "flip.h"
#include "config_param.h"
#include "commander.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ����շ����ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define FLIP_RATE			RATE_500_HZ
#define MID_VAL				(180.f * FLIP_RATE)
#define MAX_FLIP_RATE		1380	/* <2000 */
#define DELTA_VAL			(30000.f/MAX_FLIP_RATE)

#define FLIP_TIMEOUT		800			/*�������̳�ʱʱ��*/
#define SPEED_UP_TIMEOUT	400			/*����������ʱʱ��*/
#define REVER_SPEEDUP_TIME	160			/*�������ʱ��*/

#define FLIP_MAX_THRUST		56000		/*��󷭹�����ֵ*/

static enum
{
	FLIP_IDLE = 0,
	FLIP_SET,
	FLIP_SPEED_UP,
	FLIP_SLOW_DOWN,
	FLIP_PERIOD,
	FLIP_FINISHED,
	REVER_SPEED_UP,
	FLIP_ERROR,
}flipState = FLIP_IDLE;

enum dir_e flipDir;
static u16 step = 0;
static float vec_z = 0.45f;
static float out = 0.f;

/********************************************************
* Flyer ������� 
*********************************************************/
void flyerFlipCheck(setpoint_t* setpoint, control_t* control, state_t* state, const u32 tick)
{
	static u16 flipThrust = 0;
	static u16 tempThrust = 0;
	static u16 reverTime = 0;
	static u16 flipTimeout = 0;
	static float inter = 0.f;
	static float pitchTemp = 0.0;
	static float rollTemp = 0.0;
	static float yawTemp = 0.0;
	static float deltaThrust = 0.0;
		
	if(RATE_DO_EXECUTE(FLIP_RATE, tick)==false)
	{
		return;
	}

	switch(flipState)
	{
		case FLIP_IDLE:	/*��������״̬*/
		{
			if(flipDir!=CENTER)
			{
				if(control->thrust>25000 && state->velocity.z > -0.3f)
					flipState = FLIP_SET;
				else
					flipDir = CENTER;
			}			
			break;
		}
		case FLIP_SET:	/*��������*/
		{
			out = 0.f;
			step = 0;
			inter = 0.f;
			
			flipTimeout = 0;
			control->flipDir = flipDir;
//			flipThrust = configParam.thrustBase - 2000;
			flipThrust = -6500.0f + 1.1f * configParam.thrustBase;
			deltaThrust = configParam.thrustBase / 90.0f;
			tempThrust = flipThrust; 
			
			rollTemp = state->attitude.roll;
			pitchTemp = state->attitude.pitch;									
			yawTemp = state->attitude.yaw;
			
			flipState = FLIP_SPEED_UP;
			break;
		}
		case FLIP_SPEED_UP:	/*������������*/
		{
			if(state->velocity.z < vec_z)
			{
				setpoint->isAltHold = false;
				if(tempThrust < FLIP_MAX_THRUST)
					tempThrust += deltaThrust;
				setpoint->thrust = tempThrust;
				
				if(flipTimeout++ > SPEED_UP_TIMEOUT)	/*��ʱ����*/
				{
					flipTimeout = 0;
					flipState = FLIP_SLOW_DOWN;			/*ֱ�ӽ�����һ��״̬*/
				}														
			}else	
			{	
				flipTimeout = 0;					
				flipState = FLIP_SLOW_DOWN;
			}		
			break;
		}
		case FLIP_SLOW_DOWN:	/*���ٹ���*/
		{
			if(state->velocity.z>0.1f && tempThrust>flipThrust)
			{
//				tempThrust -= 3000;
				tempThrust -= (6500.f - flipThrust / 10.0f);
				setpoint->isAltHold = false;
				setpoint->thrust = tempThrust;
			}else
			{
				flipState = FLIP_PERIOD;
			}
		}
		case FLIP_PERIOD:	/*��������*/
		{
			if(flipTimeout++ > FLIP_TIMEOUT)	/*��ʱ����*/
			{
				flipTimeout = 0;
				flipState = FLIP_ERROR;
			}
			
			setpoint->isAltHold = false;
			setpoint->thrust = flipThrust - 3*out;
			
			{
				inter += out;
				if(inter < MID_VAL)
				{
					if(out < MAX_FLIP_RATE)
						out += DELTA_VAL;
					else
						step++;					
				}else
				{
					if(step > 5)
					{						
						step--;			
					}else
					{
						if(out>DELTA_VAL && inter < 2*MID_VAL)						
							out -= DELTA_VAL;
						else							
							flipState = FLIP_FINISHED;						
					}
				}
			}
			switch(control->flipDir)	
			{
				case FORWARD:	/* pitch+ */
					setpoint->attitude.pitch = out;	
					setpoint->attitude.roll = state->attitude.roll = rollTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;				
				case BACK:		/* pitch- */
					setpoint->attitude.pitch = -out;
					setpoint->attitude.roll = state->attitude.roll = rollTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case LEFT:		/* roll- */
					setpoint->attitude.roll = -out;	
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case RIGHT:		/* roll+ */					
					setpoint->attitude.roll = out;
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				default :break;
			}
			break;
		}
		case FLIP_FINISHED:	/*�������*/
		{
			setpoint->isAltHold = false;
			setpoint->thrust = tempThrust;
			tempThrust = flipThrust;
			
			reverTime = 0;
			flipTimeout = 0;
			flipDir = CENTER;	
			control->flipDir = flipDir;

			flipState = REVER_SPEED_UP;
			break;
		}
		case REVER_SPEED_UP:	/*������ɺ� �������*/
		{
			if(reverTime++<REVER_SPEEDUP_TIME)	
			{
				if(tempThrust < FLIP_MAX_THRUST)
					tempThrust += 2.0f * deltaThrust;
				setpoint->isAltHold = false;
				setpoint->thrust = tempThrust;
			}else
			{				
				flipTimeout = 0;
				flipState = FLIP_IDLE;							
			}
			break;
		}
		case FLIP_ERROR:
		{
			reverTime = 0;
			flipDir = CENTER;	
			control->flipDir = CENTER;
			
			setpoint->isAltHold = false;
			setpoint->thrust = 0;
			if(flipTimeout++ > 5)
			{
				flipTimeout = 0;
				flipState = FLIP_IDLE;
			}
			break;
		}
		default : break;
	}			
}


//���÷�������
void setFlipDir(u8 dir)
{
	flipDir = (enum dir_e)dir;	
}

