
�����������ص�ַ:
	http://www.openedv.com/thread-105197-1-1.html

MiniFly����:

         HEAD
	  M4  ��  M1
	   \     /
		\   /
		 \ /
		 / \
		/   \
	   /     \
	  M3     M2
	
Ӳ����Դ:
	1,MCU:STM32F411CEU6 (FLAH:512K, RAM:128K, ϵͳ����ʱ��Ƶ��:96MHz)
	2,9��MPU9250������IIC1��(IMU_SCL:PB8, IMU_SDA:PB9, ͨ�ŷ�ʽ:ģ��IIC) 
	3,��ѹ��BMP280������MPU9250�ĸ���IIC��(AUX_DA,AUX_CL)
	4,����ͨ��NFR51822������UART2��(NRF_RX:PA2, NRF_TX:PA3, NRF_FLOW_CTRL:PA0) 
	5,MOTOR1������TIM4_CH2��(PB7)
	6,MOTOR2������TIM4_CH1��(PB6)
	7,MOTOR3������TIM2_CH3��(PB10)
	8,MOTOR4������TIM2_CH1��(PA5)
	9,LED_BLUE_L������PB12��	(MORTOR3��Ӧ����ɫLED, �ߵ�ƽ��Ч)
	10,LED_GREEN_L������PA6��	(MORTOR4��Ӧ����ɫLED, �͵�ƽ��Ч)
	11,LED_RED_L������PA7��		(MORTOR4��Ӧ�ĺ�ɫLED, �͵�ƽ��Ч)
	12,LED_GREEN_R������PC13��	(MORTOR1��Ӧ����ɫLED, �͵�ƽ��Ч)
	13,LED_RED_R������PC14��	(MORTOR1��Ӧ�ĺ�ɫLED, �͵�ƽ��Ч)
	14,��չIIC�ӿ�(SDA:PB4, SCL:PB5) 
	15,��չSPI2�ӿ�(SCK:PB13, MISO:PB14, MOSI:PB15)  
	16,��չUART1�ӿ�(RX1:PB3, TX1:PA15, �������ͷģ�����ô˽ӿ�)  
	17,��չGPIO(CS0:PC15, CS1:PB0, CS2:PB1, CS3:PA8). 	
	18,USB_SLAVE�ӿ�(USB_ID:PA10, USB_DM:PA11, USB_DP:PA12)

ʵ������:
	MiniFly������MOTOR1~4�����20%��ռ�ձ�����ת��50ms,Ȼ��رյ��; 
	
	����:
		LED_BLUE_L:���ָʾ�ƣ����ʱ1S������˸,ֹͣ������;
		LED_GREEN_L:����ͨ�����ݽ���ָʾ�ƣ�����һ������һ��;
		LED_RED_L:����ͨ�����ݷ���ָʾ�ƣ�����һ������һ��;
		LED_GREEN_R:2S��������ָʾ������δУ׼��У׼��ɺ�0.5S���ڿ���;
		LED_RED_R:������ʾ��ش��ڵ͵���״̬����ֹͣ���У�Ȼ�����س��;
		LED_RED_L��LED_RED_Rͬʱ��������ʾMiniFly�������״̬; 

ע������:
	�������غ͵���ǰ���뽫���������ز���STM32����
	Bootloader��ʼ��ַ(BOOTLOADER_ADDR) 0x08000000;
	�̼���ʼ��ַ(FIRMWARE_START_ADDR) 	0x08008000;


�̼����¼�¼:
	Firmware V1.0 Release(Ӳ���汾:V1.32, DATE:2017-06-30)
	
	
	Firmware V1.1 Release(Ӳ���汾:V1.32��V1.4, DATE:2017-10-20)
		1.MCU�ʹ�����ͨ�Ų���Ӳ��IIC,��ȡ�����������жϷ�ʽ�����ݶ�ȡ���ʸ���;
		2.��������������sensorsTask�����ڶ�ȡ�ʹ����������ݣ�����Ĵ��������ݷ�����У��ȴ���ʹ�ã�����stabilizerTask���񸺵�;
		3.�����Ǻͼ��ټƲ���2�׵�ͨ�˲����˲�Ч�����ã������񶯣����и��ȶ�;
		4.������չģ������expModuleMgtTask���Լ��ļ�����EXP_MODULES,���ڹ�����չģ��;
		5.��չ�ӿ�����wifi����ģ��ATK-WIFI-MODULE��֧���ֻ���ƽ�壩wifi����MiniFly(���У����գ�¼��);
		6.��չ�ӿ����ӿ���RGB�ƻ�ģ��ATK-LED-RING,���ṩ10�ֵƻ�Ч���������ԣ��û����Կ����Լ�ϲ���ĵƻ�Ч��;
		7.ÿ����չģ�鶼���Լ���ID��������HARDWARE����������module_detect.c,���ڼ��MiniFly���ص���չģ��;
		8.��չģ���ʹ��˵����ο�ģ���û��ֲ�;







					����ԭ��@ALIENTEK
					2017-6-30
					������������ӿƼ����޹�˾
					�绰��020-38271790
					���棺020-36773971
					����http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					��˾��վ��www.alientek.com
					������̳��www.openedv.com
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
















