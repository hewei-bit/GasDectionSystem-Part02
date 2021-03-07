#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "usart2.h"
//#include "usart3.h"
#include "led.h"
#include "dht11.h"
#include "oled.h"
//#include "rc522.h"
#include "cjson.h"
#include "cJson_test.h"
//#include "lora.h"
#include "includes.h"

/****************************************************************
*��    ��:����վ������й©���ϵͳ
*��    ��:��ε
*��������:2020/10/14 ����ϵͳ���
*��ǰ����:2020/11/16 ���V1.0
*��ǰ����:2020/12/24 ���V2.0
*��ǰ����:2021/01/08 ���V3.0
*��ǰ����:2021/01/18 ���V4.0
*��ǰ����:2021/03/04 δ���V4.0
*����
	1.��ʼ���� �����ź��� ��Ϣ���� ������ �¼���־�� ����
	2.DHT11��ʪ�Ȳɼ�,�����߲ɼ���  	������ oled��ʾ		����ͨ����Ϣ���з������ݵ��̴߳洢��ת������
	3.TDLAS����Ũ�Ȳɼ������ڽ���Ũ�����ݣ������� oled��ʾ	����ͨ����Ϣ���з������ݵ��̴߳洢��ת������        
	4.MQ135 Ũ�Ȳɼ�
	5.MQ4 Ũ�Ȳɼ�
	6.���Ź�
	7.���ش洢���� �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
	8.LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
	9.RTCʱ����ʾ ������ oled��ʾ	
	10.LED0 �ź����������� ����
	11.LED1 ϵͳ������ʾ
	12.BEEP �ź����������� ����
	13.KEY 
	14.����״̬
	
	
*˵  ��:		
	��ǰ���뾡����ʵ����ģ�黯��̣�һ���������һ��Ӳ������򵥵�
	led�����������ɵ����������
*****************************************************************/

//V1.0 ���������ں˴��� ���������ݲɼ� json���ݷ�װ
//V2.0 ���OLED��ʾ ������Ӧ  LORA������dht11��TDLAS����Ϣ���д���
//V3.0 �����λ������λ����������ѯ
//V4.0 ���lora��Ϣ���� 


/*****************************���������ջ*************************************/
//UCOSIII���������ȼ��û�������ʹ�ã�ALIENTEK
//����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
//���ȼ�0���жϷ������������� OS_IntQTask()
//���ȼ�1��ʱ�ӽ������� OS_TickTask()
//���ȼ�2����ʱ���� OS_TmrTask()
//���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
//���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()

//����1 ��ʼ����
#define START_TASK_PRIO		3	
#define START_STK_SIZE 		512
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

//����2 DHT11��ʪ�Ȳɼ� 
#define DHT11_TASK_PRIO 	4
#define DHT11_STK_SIZE		128
OS_TCB DHT11_Task_TCB;
CPU_STK DHT11_TASK_STK[DHT11_STK_SIZE];
void DHT11_task(void *parg);

//����3 TDLAS ����Ũ�Ȳɼ�	usart2
#define TDLAS_TASK_PRIO 	5
#define TDLAS_STK_SIZE		512
OS_TCB TDLAS_Task_TCB;
CPU_STK TDLAS_TASK_STK[TDLAS_STK_SIZE];
void TDLAS_task(void *parg);

//����4  mq135 ����Ũ�Ȳɼ�	
#define MQ135_TASK_PRIO 	5
#define MQ135_STK_SIZE		128
OS_TCB MQ135_Task_TCB;
CPU_STK MQ135_TASK_STK[MQ135_STK_SIZE];
void MQ135_task(void *parg);

//����5 mq4 ����Ũ�Ȳɼ�	
#define MQ4_TASK_PRIO 	5
#define MQ4_STK_SIZE		128
OS_TCB MQ4_Task_TCB;
CPU_STK MQ4_TASK_STK[MQ4_STK_SIZE];
void MQ4_task(void *parg);

//����6 ���Ź� 
#define IWG_TASK_PRIO 		6
#define IWG_STK_SIZE		128
OS_TCB IWG_Task_TCB;
CPU_STK IWG_TASK_STK[IWG_STK_SIZE];
void IWG_task(void *parg);

//����7 �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
#define SAVE_TASK_PRIO 		9
#define SAVE_STK_SIZE		512
OS_TCB SAVE_Task_TCB;
CPU_STK SAVE_TASK_STK[SAVE_STK_SIZE];
void SAVE_task(void *parg);

//����8 LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
#define LORA_TASK_PRIO 		8
#define LORA_STK_SIZE		512
OS_TCB LORA_Task_TCB;
CPU_STK LORA_TASK_STK[LORA_STK_SIZE];
void LORA_task(void *parg);

//����9 rtcʱ����ʾ	������ oled��ʾ	
#define RTC_TASK_PRIO 		9
#define RTC_STK_SIZE		128
OS_TCB RTC_Task_TCB;
CPU_STK RTC_TASK_STK[RTC_STK_SIZE];
void RTC_task(void *parg);

//����10 LED0���� 
#define LED0_TASK_PRIO		10
#define LED0_STK_SIZE 		128
OS_TCB Led0TaskTCB;
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

//����11 LED1����
#define LED1_TASK_PRIO		11	
#define LED1_STK_SIZE 		128
OS_TCB Led1TaskTCB;
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
void led1_task(void *p_arg);

//����12.BEEP
#define BEEP_TASK_PRIO		12
#define BEEP_STK_SIZE 		128
OS_TCB BEEP_Task_TCB;
CPU_STK BEEP_TASK_STK[BEEP_STK_SIZE];
void BEEP_task(void *p_arg);

//����13.key
#define KEY_TASK_PRIO		13
#define KEY_STK_SIZE 		128
OS_TCB KEY_Task_TCB;
CPU_STK KEY_TASK_STK[KEY_STK_SIZE];
void KEY_task(void *p_arg);

//����14.����״̬
#define TASK_STA_TASK_PRIO		20
#define TASK_STA_STK_SIZE		128
OS_TCB	TASK_STA_Task_TCB;
CPU_STK	TASK_STA_TASK_STK[TASK_STA_STK_SIZE];
void TASK_STA_task(void *p_arg);


//����15 ����������ʾ
#define FLOAT_TASK_PRIO		6
#define FLOAT_STK_SIZE		128
OS_TCB	FloatTaskTCB;
__align(8) CPU_STK	FLOAT_TASK_STK[FLOAT_STK_SIZE];
void float_task(void *p_arg);


/*****************************�¼���־��Ķ���******************************/
OS_FLAG_GRP				g_flag_grp;			

/*******************************�ź����Ķ���******************************/
OS_SEM					g_sem_led;		
OS_SEM					g_sem_beep;			

/*******************************�������Ķ���******************************/
OS_MUTEX				g_mutex_printf;	
OS_MUTEX				g_mutex_oled;		
OS_MUTEX				g_mutex_lcd;
OS_MUTEX				g_mutex_TDLAS;
OS_MUTEX				g_mutex_DHT11;
OS_MUTEX				g_mutex_NODE;

/*****************************��Ϣ���еĶ���*******************************/
OS_Q	 				g_queue_usart1;	
OS_Q	 				g_queue_usart2;				
OS_Q	 				g_queue_usart3;				

OS_Q					g_queue_dht11_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_dht11_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_TDLAS_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_TDLAS_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_MQ135_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_MQ135_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_MQ4_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_MQ4_to_txt;			//��Ϣ���еĶ���

#define CORE_OBJ_NUM	2	//�ں˶��������һ��3����2���ź�����һ����Ϣ����						

uint32_t 				g_oled_display_flag=1;
uint32_t 				g_oled_display_time_count=0;

//�ڵ�ṹ��
struct NODE
{
	int device_id;
	int lora_address;
	int lora_channel;
	char* temperature;
	char* humidity;
	char* CH4concentration;
} *node1,node_1;


//���DHT11��tdlas����������
char temp_buf[16] = {0};
char humi_buf[16] = {0};
char TDLAS[20] = {0};


#define DEBUG_PRINTF_EN	1
void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	
	OS_ERR err;
	
	va_list args;
	va_start(args, format);
	
	OSMutexPend(&g_mutex_printf,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
	vprintf(format, args);
	OSMutexPost(&g_mutex_printf,OS_OPT_POST_NONE,&err);
	
	va_end(args);
#else
	(void)0;
#endif
}


int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	/**************************Ӳ����ʼ��*************************/
	delay_init();       //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
	uart_init(115200);    //���ڲ���������
	usart2_init(115200);    //���ڲ���������
	LED_Init();         //LED��ʼ��
	OLED_Init();		//��ʼ��OLED
	//RC522_Init();	    //��ʼ����Ƶ��оƬ
	
	while(DHT11_Init());//��ʪ�ȴ������ĳ�ʼ��
	
	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���
	
	//1.������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

//����1.��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
	
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	
	//�����¼���־�飬���б�־λ��ֵΪ0
	OSFlagCreate(&g_flag_grp,		"g_flag_grp",0,&err);
	
	//�����ź�������ֵΪ0����һ����Դ
	OSSemCreate(&g_sem_led,"g_sem_led",0,&err);
	OSSemCreate(&g_sem_beep,"g_sem_beep",0,&err);	
	
	//����������
	OSMutexCreate(&g_mutex_printf,	"g_mutex_printf",&err);	
	OSMutexCreate(&g_mutex_oled,	"g_mutex_oled",&err);
	OSMutexCreate(&g_mutex_lcd,		"g_mutex_olcd",&err);
	
	//������Ϣ���У�����usart2������TDLAS
	OSQCreate(&g_queue_usart1,"g_queue_usart1",16,&err);
	OSQCreate(&g_queue_usart2,"g_queue_usart2",16,&err);
	OSQCreate(&g_queue_usart2,"g_queue_usart3",16,&err);
	
	//������Ϣ���У�����dht11������lora
	OSQCreate(&g_queue_dht11_to_lora,"g_queue_dht11_to_lora",16,&err);
	OSQCreate(&g_queue_dht11_to_txt,"g_queue_dht11_to_txt",16,&err);
	
	OSQCreate(&g_queue_TDLAS_to_lora,"g_queue_TDLAS_to_lora",16,&err);
	OSQCreate(&g_queue_TDLAS_to_txt,"g_queue_TDLAS_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ135_to_lora,"g_queue_MQ135_to_lora",16,&err);
	OSQCreate(&g_queue_MQ135_to_txt,"g_queue_MQ135_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ4_to_lora,"g_queue_MQ4_to_lora",16,&err);
	OSQCreate(&g_queue_MQ4_to_txt,"g_queue_MQ4_to_txt",16,&err);

	//2.����DHT11����
	OSTaskCreate((OS_TCB 	* )&DHT11_Task_TCB,		
				 (CPU_CHAR	* )"DHT11 task", 		
                 (OS_TASK_PTR )DHT11_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )DHT11_TASK_PRIO,     	
                 (CPU_STK   * )&DHT11_TASK_STK[0],	
                 (CPU_STK_SIZE)DHT11_STK_SIZE/10,	
                 (CPU_STK_SIZE)DHT11_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);		
	
	//3.����TDLAS����
	OSTaskCreate((OS_TCB 	* )&TDLAS_Task_TCB,		
				 (CPU_CHAR	* )"TDLAS task", 		
                 (OS_TASK_PTR )TDLAS_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TDLAS_TASK_PRIO,     	
                 (CPU_STK   * )&TDLAS_TASK_STK[0],	
                 (CPU_STK_SIZE)TDLAS_STK_SIZE/10,	
                 (CPU_STK_SIZE)TDLAS_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//4.����mq135����
	OSTaskCreate((OS_TCB 	* )&MQ135_Task_TCB,		
				 (CPU_CHAR	* )"MQ135 task", 		
                 (OS_TASK_PTR )MQ135_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MQ135_TASK_PRIO,     	
                 (CPU_STK   * )&MQ135_TASK_STK[0],	
                 (CPU_STK_SIZE)MQ135_STK_SIZE/10,	
                 (CPU_STK_SIZE)MQ135_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//5.����mq4����
	OSTaskCreate((OS_TCB 	* )&MQ4_Task_TCB,		
				 (CPU_CHAR	* )"MQ4 task", 		
                 (OS_TASK_PTR )MQ4_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MQ4_TASK_PRIO,     	
                 (CPU_STK   * )&MQ4_TASK_STK[0],	
                 (CPU_STK_SIZE)MQ4_STK_SIZE/10,	
                 (CPU_STK_SIZE)MQ4_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
		
	//7.����SAVE����
	OSTaskCreate((OS_TCB 	* )&SAVE_Task_TCB,		
				 (CPU_CHAR	* )"SAVE task", 		
                 (OS_TASK_PTR )SAVE_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )SAVE_TASK_PRIO,     	
                 (CPU_STK   * )&SAVE_TASK_STK[0],	
                 (CPU_STK_SIZE)SAVE_STK_SIZE/10,	
                 (CPU_STK_SIZE)SAVE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//8.����LORA����
	OSTaskCreate((OS_TCB 	* )&LORA_Task_TCB,		
				 (CPU_CHAR	* )"LORA task", 		
                 (OS_TASK_PTR )LORA_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LORA_TASK_PRIO,     	
                 (CPU_STK   * )&LORA_TASK_STK[0],	
                 (CPU_STK_SIZE)LORA_STK_SIZE/10,	
                 (CPU_STK_SIZE)LORA_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//9.����RTC����
	OSTaskCreate((OS_TCB 	* )&RTC_Task_TCB,		
				 (CPU_CHAR	* )"RTC task", 		
                 (OS_TASK_PTR )RTC_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )RTC_TASK_PRIO,     	
                 (CPU_STK   * )&RTC_TASK_STK[0],	
                 (CPU_STK_SIZE)RTC_STK_SIZE/10,	
                 (CPU_STK_SIZE)RTC_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//10.����LED0����
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//11.����LED1����
	OSTaskCreate((OS_TCB 	* )&Led1TaskTCB,		
				 (CPU_CHAR	* )"led1 task", 		
                 (OS_TASK_PTR )led1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED1_TASK_PRIO,     	
                 (CPU_STK   * )&LED1_TASK_STK[0],	
                 (CPU_STK_SIZE)LED1_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
	
	//12.����BEEP����
	OSTaskCreate((OS_TCB 	* )&BEEP_Task_TCB,		
				 (CPU_CHAR	* )"BEEP task", 		
                 (OS_TASK_PTR )BEEP_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )BEEP_TASK_PRIO,     	
                 (CPU_STK   * )&BEEP_TASK_STK[0],	
                 (CPU_STK_SIZE)BEEP_STK_SIZE/10,	
                 (CPU_STK_SIZE)BEEP_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			 
	
	//13.����KEY����
	OSTaskCreate((OS_TCB 	* )&KEY_Task_TCB,		
				 (CPU_CHAR	* )"KEY task", 		
                 (OS_TASK_PTR )KEY_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )KEY_TASK_PRIO,     	
                 (CPU_STK   * )&KEY_TASK_STK[0],	
                 (CPU_STK_SIZE)KEY_STK_SIZE/10,	
                 (CPU_STK_SIZE)KEY_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
				 
	//14.����TASK_STA����
	OSTaskCreate((OS_TCB 	* )&TASK_STA_Task_TCB,		
				 (CPU_CHAR	* )"TASK_STA task", 		
                 (OS_TASK_PTR )TASK_STA_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_STA_TASK_PRIO,     	
                 (CPU_STK   * )&TASK_STA_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK_STA_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_STA_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//15.���������������
	OSTaskCreate((OS_TCB 	* )&FloatTaskTCB,		
				 (CPU_CHAR	* )"float test task", 		
                 (OS_TASK_PTR )float_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )FLOAT_TASK_PRIO,     	
                 (CPU_STK   * )&FLOAT_TASK_STK[0],	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE/10,	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	

	
				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�����ٽ���
}
void json_test(struct NODE* node)
{
	/* Build the JSON array [[1, 2], {"cool": true}] */
	/* print the version */
	cJSON *root = NULL;
	cJSON *Node = NULL;
	char *out = NULL;
	char *end = "#"; //����β����ʶ��
	//printf("Version: %s\n", cJSON_Version());

	/* Now some samplecode for building objects concisely: */	
	root = cJSON_CreateObject();
	cJSON_AddStringToObject(root,"reason",(const char *)"success");
	cJSON_AddItemToObject(root,"node_1",Node = cJSON_CreateObject());
	cJSON_AddNumberToObject(Node,"DeviceID",node->device_id);
	cJSON_AddNumberToObject(Node,"LORA_ADD",node->lora_address);
	cJSON_AddNumberToObject(Node,"LORA_CHN",node->lora_channel);
	cJSON_AddStringToObject(Node, "Temp", node->temperature);
	cJSON_AddStringToObject(Node, "humi", node->humidity);
	cJSON_AddStringToObject(Node, "CH4_concentration", (const char *)node->CH4concentration);

	/* Print to text */
//	if (print_preallocated(root) != 0) {
//		cJSON_Delete(root);
//		return ;
//	}

	out = cJSON_Print(root); 
	strcat(out,end);//�������β����ʶ��
	dgb_printf_safe("%s",out);
	
	cJSON_Delete(root);	
	free(out);
		
}

//����2 DHT11��ʪ�Ȳɼ� 
void DHT11_task(void *p_arg)
{
	OS_ERR err;
	
	//DHT11 ��ʪ��
	uint8_t dht11_data[5] = {0};
	char buf[16] = {0};
	//����
	dgb_printf_safe("DHT11 task running\r\n");
	
	//LCD_ShowString(30,130,200,16,16,"DHT11 OK");
	//POINT_COLOR=BLUE;//��������Ϊ��ɫ 
	
	while(1)
	{
		//��ȡ��ʪ��ֵ							  
		DHT11_Read_Data(dht11_data);	
		//������Ϣ��LORA����
		OSQPost((OS_Q*		)&g_queue_dht11_to_lora,		
				(void*		)dht11_data,
				(OS_MSG_SIZE)sizeof(dht11_data),
				(OS_OPT		)OS_OPT_POST_FIFO,
				(OS_ERR*	)&err);
				
		//���͸�txt����
		//OSQPost(&g_queue_dht11_to_txt,
				//(void *)dht11_data,
				//sizeof(dht11_data),
				//OS_OPT_POST_FIFO,
				//&err);
		
		//��ϴ���
		sprintf((char *)buf,"T:%02d.%dC H:%02d.%d%%",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
		//sprintf((char *)temp_buf,"%02d.%d",dht11_data[2],dht11_data[3]);
		//sprintf((char *)humi_buf,"%02d.%d",dht11_data[0],dht11_data[1]);
		
		//������usart1,���е���
		//dgb_printf_safe("%s\r\n",buf);

#if 0					
		//lcd��ʾ
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,150,100,16,16,temp_buf);
		LCD_ShowString(30,170,100,16,16,humi_buf);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);		
#endif	 

		//OLED��ʾ
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,2,buf,16);		
		OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);

		//��ʱ�����������
		delay_ms(4000);

	}
}

//����3TDLAS����Ũ�Ȳɼ������ڽ���Ũ�����ݣ������� oled��ʾ	����ͨ����Ϣ���з������ݵ��̴߳洢��ת������
void TDLAS_task(void *p_arg)
{
	OS_ERR err;	
	
	int i = 0;
	uint8_t TDLAS_buf[30];
	uint8_t *TDLAS_res=NULL;
	OS_MSG_SIZE TDLAS_size;
	dgb_printf_safe("TDLAS task running\r\n");
	
	while(1)
	{		
		
#if 1	
		//�ȴ���Ϣ���з������ʹ���
		TDLAS_res = OSQPend((OS_Q*			)&g_queue_usart2,
							(OS_TICK		)0,
							(OS_OPT			)OS_OPT_PEND_BLOCKING,
							(OS_MSG_SIZE*	)&TDLAS_size,
							(CPU_TS*		)NULL,
							(OS_ERR*		)&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[TDLAS_task_usart2][OSQPend]Error Code = %d\r\n",err);		
		}
		
		if(USART2_RX_STA&0x8000)
		{                                           
			int len=USART2_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
			//dgb_printf_safe("len: %d \r\n",len);
			for(i = 0;i < len;i++)
			{
				TDLAS[i] = USART2_RX_BUF[i];
			}
			//dgb_printf_safe("TDLAS:%s\r\n",USART2_RX_BUF);
			//dgb_printf_safe("TDLAS:%s\r\n",TDLAS);
			USART2_RX_STA = 0;
		}	
#endif 
		//��ʱ�����������
		delay_ms(1000);
	}
}


//����4  mq135 ����Ũ�Ȳɼ�	
void MQ135_task(void *parg)
{
	OS_ERR err;
	//dgb_printf_safe("MQ135 task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}

//����5 mq4 ����Ũ�Ȳɼ�	
void MQ4_task(void *parg)
{
	OS_ERR err;
	//dgb_printf_safe("MQ4 task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}

//����6 ���Ź� 
void IWG_task(void *parg)
{
	OS_ERR err;
	//dgb_printf_safe("IWG task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}

//����7 �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
void SAVE_task(void *parg)
{
	OS_ERR err;
	//dgb_printf_safe("SAVE task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}

//����8 LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
void LORA_task(void *p_arg)
{
	OS_ERR err;  
	int i = 0;
	char *dht11_data_recv = NULL;
	OS_MSG_SIZE dht11_data_size;
	char buf[16] = {0};
	char USART_buf[30];
	OS_MSG_SIZE USART_size;
	char *USART_res = NULL;
	
	//�ȴ�����ں˶���
	OS_OBJ_QTY index;
	OS_PEND_DATA pend_multi_tbl[CORE_OBJ_NUM];	
	pend_multi_tbl[0].PendObjPtr=(OS_PEND_OBJ*)&g_queue_usart1;
	pend_multi_tbl[1].PendObjPtr=(OS_PEND_OBJ*)&g_queue_dht11_to_lora;
	
	node1 = (struct NODE*)malloc(sizeof(struct NODE));
	
	dgb_printf_safe("LORA task running\r\n");
	
	while(1)
	{
#if 1	
		//�ȴ�����ں˶���
		index = OSPendMulti((OS_PEND_DATA*	)pend_multi_tbl,		//�ں˶�������	
							(OS_OBJ_QTY		)CORE_OBJ_NUM,			//�ں�����
							(OS_TICK	 	)0,						//��Զ�ȴ���ȥ
							(OS_OPT        	)OS_OPT_PEND_BLOCKING,
							(OS_ERR*		)&err);
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_lora_dht11][OSQPend]Error Code = %d\r\n",err);
			continue;
		}
		//������������ź���
		if(pend_multi_tbl[0].RdyObjPtr == (OS_PEND_OBJ *)(&g_queue_usart1)){
			//dgb_printf_safe("\r\ncheck\r\n");
			if(USART_RX_STA&0x8000)
			{                                           
				int len=USART_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
				//dgb_printf_safe("usart1_len: %d \r\n",len);
				//dgb_printf_safe("TDLAS:%s\r\n",USART_RX_BUF);
					
				if(strstr((const char *)USART_RX_BUF,"check"))
				{
					OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);
					json_test(node1);
					OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
				}
				
				USART_RX_STA = 0;
			}	
		}
		else if(pend_multi_tbl[1].RdyObjPtr == (OS_PEND_OBJ *)(&g_queue_dht11_to_lora))
		{
			dht11_data_recv = pend_multi_tbl[1].RdyMsgPtr;
			sprintf((char *)buf,"T:%02d.%02dC H:%02d.%02d%%",dht11_data_recv[2],dht11_data_recv[3],dht11_data_recv[0],dht11_data_recv[1]);
			sprintf((char *)temp_buf,"%02d.%d",dht11_data_recv[2],dht11_data_recv[3]);
			sprintf((char *)humi_buf,"%02d.%d",dht11_data_recv[0],dht11_data_recv[1]);
			//dgb_printf_safe("%s\r\n",buf);
			//dgb_printf_safe("\r\n�յ���Ϣ����\r\n");

			OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			node1->device_id = 1;
			node1->lora_address = 10;
			node1->lora_channel = 23;
			node1->humidity = humi_buf;
			node1->temperature = temp_buf;
			node1->CH4concentration = TDLAS;
			//json_test(node1);
			OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
			
			memset((void *)buf,0,sizeof(buf));
			//memset((void *)temp_buf,0,sizeof(temp_buf));
			//memset((void *)humi_buf,0,sizeof(humi_buf));
		}
#endif	
	
#if 0
		//�ȴ�dht11��Ϣ����
		dht11_data_recv = OSQPend((OS_Q*			)&g_queue_dht11_to_lora,   
								(OS_TICK		)0,
								(OS_OPT			)OS_OPT_PEND_BLOCKING,
								(OS_MSG_SIZE*	)&dht11_data_size,		
								(CPU_TS*		)0,
								(OS_ERR*		)&err);
		//��ȷ���յ���Ϣ
		if(dht11_data_recv && dht11_data_size)
		{
			//dgb_printf_safe("recv from dht11 %d\r\n",err);
			//��ϴ���
			//sprintf((char *)buf,"T:%02d.%dC H:%02d.%d%%",dht11_data_recv[2],dht11_data_recv[3],dht11_data_recv[0],dht11_data_recv[1]);
			//sprintf((char *)temp_buf,"%02d.%d",dht11_data_recv[2],dht11_data_recv[3]);
			//sprintf((char *)humi_buf,"%02d.%d",dht11_data_recv[0],dht11_data_recv[1]);
			//sprintf((char *)CH4_buf,"5000");	
			
			node1->device_id = 10;
			node1->lora_address = 10;
			node1->lora_channel = 23;
			node1->humidity = humi_buf;
			node1->temperature = temp_buf;
			node1->CH4concentration = TDLAS;
			
			//������usart1,���е���
			//dgb_printf_safe("%s\r\n",buf);
			json_test(node1);
		
			memset((void *)buf,0,sizeof(buf));
			memset((void *)temp_buf,0,sizeof(temp_buf));
			memset((void *)humi_buf,0,sizeof(humi_buf));
			memset((void *)CH4_buf,0,sizeof(CH4_buf));
		}		
#endif

#if 0	
		//�ȴ���Ϣ����
		USART_res = OSQPend((OS_Q*			)&g_queue_usart1,
							(OS_TICK		)0,
							(OS_OPT			)OS_OPT_PEND_BLOCKING,
							(OS_MSG_SIZE*	)&USART_size,
							(CPU_TS*		)NULL,
							(OS_ERR*		)&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[_task_usart][OSQPend]Error Code = %d\r\n",err);		
		}
		
		if(USART_RX_STA&0x8000)
		{                                           
			int len=USART_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���	
			//dgb_printf_safe("usart1_len: %d \r\n",len);
			//dgb_printf_safe("TDLAS:%s\r\n",USART_RX_BUF);
		
			if(strstr((const char *)USART_RX_BUF,"check"))
			{
				json_test(node1);
			}
			
			USART_RX_STA = 0;
		}	
#endif 
			
		delay_ms(1000);
		
	}
}
//����9 rtcʱ����ʾ	������ oled��ʾ	
void RTC_task(void *parg)
{
	OS_ERR err;
	//dgb_printf_safe("BEEP task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}


//����10.ϵͳ������ʾ led0������
void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		LED0=0;
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		LED0=1;
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms
	}
}

//����11.�ⱨ������ led1������
void led1_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		LED1=~LED1;
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms
	}
}

//����12.������������
void BEEP_task(void *p_arg)
{
	OS_ERR err;
	//dgb_printf_safe("BEEP task running\r\n");
	
	while(1)
	{
		delay_ms(100);
	}
}

//����13.��������
void KEY_task(void *p_arg)
{
	OS_ERR err;

	OS_FLAGS flags=0;
	
	//dgb_printf_safe("KEY task running\r\n");
	
	while(1)
	{
#if 0
		//һֱ�����ȴ��¼���־��1���ȴ��ɹ��󣬽���Ӧ��0
		flags = OSFlagPend(&g_flag_grp,FLAG_GRP_KEY0_DOWN
									|FLAG_GRP_KEY1_DOWN
									|FLAG_GRP_KEY2_DOWN
									|FLAG_GRP_WK_UP_DOWN,
									0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			//dgb_printf_safe("[task1][OSFlagPend]Error Code = %d\r\n",err);
			continue;
		}
		
		//WK_UP����
		if(flags & FLAG_GRP_WK_UP_DOWN){	
			//��ֹEXTI0�����ж�
			NVIC_DisableIRQ(EXTI0_IRQn);
			if(WK_UP == 1){
				delay_ms(20);//����
				if(WK_UP == 1){
					//dgb_printf_safe("WK_UP happend\r\n");
					//�ȴ�����WK_UP�ͷ�
					while(WK_UP == 1){
						delay_ms(1);
					}						
					
					BEEP = !BEEP;
				}
			}	
			//����EXTI0�����ж�
			NVIC_EnableIRQ(EXTI0_IRQn);	
			//���EXTI0�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line0);			
		}
		
		//KEY2����
		if(flags & FLAG_GRP_KEY2_DOWN){	
			if(KEY2 == 0){
				delay_ms(20);//����
				if(KEY2 == 0){
					//dgb_printf_safe("key2 happend\r\n");
					//�ȴ�����2�ͷ�
					while(KEY2==0){
						delay_ms(1);
					}	
					
					LED0 = !LED0;
				}
			}	
			//���EXTI2�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line2);			
		}
		
		//KEY1����
		if(flags & FLAG_GRP_KEY1_DOWN){	
			if(KEY1 == 0){
				delay_ms(20);//����
				if(KEY1 == 0){
					//dgb_printf_safe("key1 happend\r\n");
					//�ȴ�����1�ͷ�
					while(KEY1==0){
						delay_ms(1);
					}	
					//������Ϣ
//					OSSemPost(&g_json,OS_OPT_POST_1,&err);//�����ź���
				}
			}	
			//���EXTI3�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line3);			
		}
		
//		//KEY0����
//		if(flags & FLAG_GRP_KEY0_DOWN){	
//			if(KEY0 == 0){
//				delay_ms(20);//����
//				if(KEY0 == 0){
//					dgb_printf_safe("key0 happend\r\n");
//					//�ȴ�����0�ͷ�
//					while(KEY0==0){
//						delay_ms(1);
//					}		
//					
//					LED1 = !LED1;
//					LED0 = !LED0;
//				}
//			}	
//			//���EXTI4�жϱ�־λ
//			EXTI_ClearITPendingBit(EXTI_Line4);			
//		}
#endif		
		delay_ms(1000);
	}
}

//����14.ϵͳ�ڴ�ռ�ü���
void TASK_STA_task(void *p_arg)
{
	OS_ERR err;  
	
	CPU_STK_SIZE free,used; 
	
	//dgb_printf_safe("TASK_STA task running\r\n");
	
	delay_ms(3000);
	
	while(1)
	{

#if 0
		OSTaskStkChk (&DHT11_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_ir    stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free)); 
	
		OSTaskStkChk (&LORA_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_key   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));
	
		OSTaskStkChk (&RTC_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_usart1   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));	
		
		OSTaskStkChk (&SAVE_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_mpu6050  stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));			

	
		
		OSTaskStkChk (&LED0_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_rtc   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));  	  			  

		OSTaskStkChk (&LED1_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_dht11 stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));  

		OSTaskStkChk (&BEEP_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("BEEP_task   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));

		OSTaskStkChk (&TASK_STA_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_sta   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));
#endif		
		delay_ms(6000);
	}
}

//����15.�����������
void float_task(void *p_arg)
{
	CPU_SR_ALLOC();
	static float float_num=0.01;
	while(1)
	{
		
		//RC522_Handel();
		
		
		
		
#if 0
		float_num+=0.01f;
		OS_CRITICAL_ENTER();	//�����ٽ���
		printf("float_num��ֵΪ: %.4f\r\n",float_num);
		OS_CRITICAL_EXIT();		//�˳��ٽ���
#endif	
		delay_ms(500);			//��ʱ500ms
	}
}
