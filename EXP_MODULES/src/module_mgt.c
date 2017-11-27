#include "module_mgt.h"
#include "module_detect.h"
#include "ledring12.h"
#include "wifi_ctrl.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��չģ�������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static xTimerHandle timer;
static enum expModuleID moduleID = NO_MODULE;

//��ȡ��չģ��ID
enum expModuleID getModuleID(void)
{
	return moduleID;
}

//��չģ���Դ����
void expModulePower(bool state)
{
	if(state)
	{
		wifiPowerControl(true);
		ledringPowerControl(true);
	}
	else
	{
		wifiPowerControl(false);
		ledringPowerControl(false);
	}
}

//�����ʱ���жϼ����չģ��
static void expModuleDetect(xTimerHandle xTimer)
{
	enum expModuleID id = getModuleDriverID();
	if(id != moduleID)
	{
		if(id == LED_RING)
		{
			ledring12Init();
		}
		else if(id == CAMERA)
		{
			wifiModuleInit();
		}
		moduleID = id;
	}
}

//��չģ���������
void expModuleMgtTask(void* param)
{
	timer = xTimerCreate( "expModuleTimer", 1000, pdTRUE, NULL, expModuleDetect);
	xTimerStart(timer, portMAX_DELAY);
	while(1)
	{	
		vTaskDelay(1000);
	}
}

