#include "module_detect.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��չģ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define  ADC_SAMPLE_NUM		10	 //��������

#define  ADC_LED_RING		2048 //RGB�ƻ�ģ���R1:R2 = 4.7K :4.7K
#define  ADC_CAMERA			4095 //����ͷģ���R1 = 10K 
#define  ADC_MODULE1		1310 //ģ��1��R1:R2 = 4.7K :10K
#define  ADC_MODULE2		2786 //ģ��2��R1:R2 = 10K :4.7K

#define  ADC_MODULE_RANGE	50	 //����ģ���ѹ�仯��Χֵ

static GPIO_InitTypeDef  GPIO_InitStructure;
static enum expModuleID moduleID = NO_MODULE;
static u16 adcValue[ADC_SAMPLE_NUM];

u32 my_abs(int value)
{
	return (value >=0 ? value : -value);
}
	
void expModuleDriverInit(void)	/*��չģ��������ʼ��*/
{
	ADC_InitTypeDef  		ADC_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	DMA_InitTypeDef       	DMA_InitStructure;
	//GPIO_InitTypeDef      	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��PORTBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  	//ʹ��ADC1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//ʹ��DMAʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;					//PB1
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;				//ģ������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;        	//������������
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//��ʼ��PB1
	
	/*DMA2_Stream0 channel0 ����*/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&adcValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);//ʹ��DMA2_Stream0
	
	/*ADC1 ����*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //���ADCģʽDMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=100/4=25Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,1,ADC_SampleTime_480Cycles);//����PB1����ͨ��9
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);//ʹ��DMA
	ADC_Cmd(ADC1, ENABLE);//ʹ��ADת����
	ADC_SoftwareStartConv(ADC1);//����ת��
}

enum expModuleID getModuleDriverID(void)
{
	ADC_Cmd(ADC1, DISABLE);//�ر�ADת����
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;					//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//����ģʽ
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//��ʼ��PB1
	u8 state1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);	//��ȡ״̬1
	
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;        	//����
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//��ʼ��PB1
	u8 state2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);	//��ȡ״̬2
	
	if(state1==SET && state2==RESET)//û�м�⵽ģ�����
	{
		moduleID = NO_MODULE;
	}
	else //��⵽ģ�����
	{
		u32 sum=0;
		for(int i=0; i<ADC_SAMPLE_NUM; i++)
		{
			sum += adcValue[i];
		}
		sum = sum/ADC_SAMPLE_NUM;
		
		if(my_abs(sum-ADC_LED_RING) <= ADC_MODULE_RANGE)	
			moduleID = LED_RING;
		else if(my_abs(sum-ADC_CAMERA) <= ADC_MODULE_RANGE)		
			moduleID = CAMERA;		
		else if(my_abs(sum-ADC_MODULE1) <= ADC_MODULE_RANGE)		
			moduleID = MODULE1;
		else if(my_abs(sum-ADC_MODULE2) <= ADC_MODULE_RANGE)	
			moduleID = MODULE2;
		else
			moduleID = NO_MODULE;
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;			//PB1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	//ģ������ģʽ
		GPIO_Init(GPIOB,&GPIO_InitStructure);           //��ʼ��PB1

		ADC_Cmd(ADC1, ENABLE);//ʹ��ADת����
		ADC_SoftwareStartConv(ADC1);//����ADת����
	}
	return moduleID;
}




