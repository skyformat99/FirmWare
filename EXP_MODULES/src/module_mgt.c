#include "module_mgt.h"
#include "module_detect.h"
#include "ledring12.h"
#include "wifi_ctrl.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 扩展模块管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static xTimerHandle timer;
static enum expModuleID moduleID = NO_MODULE;

//获取扩展模块ID
enum expModuleID getModuleID(void)
{
	return moduleID;
}

//扩展模块电源控制
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

//软件定时器中断检测扩展模块
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

//扩展模块管理任务
void expModuleMgtTask(void* param)
{
	timer = xTimerCreate( "expModuleTimer", 1000, pdTRUE, NULL, expModuleDetect);
	xTimerStart(timer, portMAX_DELAY);
	while(1)
	{	
		vTaskDelay(1000);
	}
}

