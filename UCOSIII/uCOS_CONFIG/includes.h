/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  INCLUDES_MODULES_PRESENT
#define  INCLUDES_MODULES_PRESENT


/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/


#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>


/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/

#include  <os.h>


/*
*********************************************************************************************************
*                                              LIBRARIES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>

/*
*********************************************************************************************************
*                                              APP / BSP
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <bsp.h>
//#include  <bsp_int.h>

//�¼���־�飺Ӳ��ʹ�õ��ı�־λ
#define FLAG_GRP_RTC_WAKEUP					0x01
#define FLAG_GRP_RTC_ALARM_A				0x02
#define FLAG_GRP_RTC_ALARM_B				0x04

#define FLAG_GRP_KEY0_DOWN					0x10
#define FLAG_GRP_KEY1_DOWN					0x20
#define FLAG_GRP_KEY2_DOWN					0x40
#define FLAG_GRP_WK_UP_DOWN					0x80


#define MAX_CONVERTED_VALUE 	4095  	//���ת��ֵ
#define VREF					3300	//����ѹֵ
#define QUEUE_NUM		10				//��Ϣ���г���
#define CORE_OBJ_NUM	2				//�ں˶��������һ��3����2���ź�����һ����Ϣ����						


//�ڵ�ṹ��
typedef struct 
{
	char device_id[4];
	char lora_address[4];
	char lora_channel[4];
	char temperature[8];
	char humidity[8];
	char CH4concentration[8];
	char Pitch[8];
	char Roll[8];
	char Yaw[8];
	char light[4];	
	char warning[4];
	char over[4];
} NODE;


extern OS_FLAG_GRP			g_flag_grp;		

extern OS_MUTEX				g_mutex_printf;		//�������Ķ���


extern OS_Q	 				g_queue_usart1;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart2;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart3;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart4;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart5;				//��Ϣ���еĶ���


extern void dgb_printf_safe(const char *format, ...);

//����1 ��ʼ����
extern OS_TCB StartTaskTCB;

//����2 DHT11��ʪ�Ȳɼ� 
extern OS_TCB DHT11_Task_TCB;

//����3 TDLAS ����Ũ�Ȳɼ�	usart2
extern OS_TCB TDLAS_Task_TCB;

//����4  mq135 ����Ũ�Ȳɼ�	
extern OS_TCB MQ135_Task_TCB;

//����5 mq4 ����Ũ�Ȳɼ�	
extern OS_TCB MQ4_Task_TCB;

//����6 ���Ź� 
extern OS_TCB IWG_Task_TCB;

//����7 �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
extern OS_TCB SAVE_Task_TCB;

//����8 LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
extern OS_TCB LORA_Task_TCB;

//����9 rtcʱ����ʾ	������ oled��ʾ	
extern OS_TCB RTC_Task_TCB;

//����10 LED0���� 
extern OS_TCB Led0TaskTCB;

//����11 LED1����
extern OS_TCB Led1TaskTCB;

//����12.BEEP
extern OS_TCB BEEP_Task_TCB;

//����13.key
extern OS_TCB KEY_Task_TCB;

//����14 MPU6050
extern OS_TCB	Mpu6050TaskTCB;

//����15.����״̬
extern OS_TCB	TASK_STA_Task_TCB;

//����16 ����������ʾ
extern OS_TCB	FloatTaskTCB;




#endif



