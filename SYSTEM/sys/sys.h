#ifndef __SYS_H
#define __SYS_H
#include "stm32f10x.h"

#include "stdio.h"
#include "string.h"
/************************************************************************************************
  * @file     SYS.h
  * @author   2020��04��21��  ħŮ������ �Ŷ�
  *
  * @briefs   �ļ�������
  *           1�������޸ĵ�����USARTx����ֲ����, ��ʾ��ʹ�õ���USART1��PA9\PA10
  *           2������һЩ�ꡢȫ�ֺ���, ��ֲʱ����Щ�ǲ����޸ĵ�
  *
  * @note     ʹ�÷���: �� "sys.c"��"sys.h" �����ļ����Ƶ������ļ��У�Ȼ���ڹ��������ü���
              ���ļ���ԭ�Ӹ��sys�ļ������ϸĽ���д���������滻�����оɵ�sys�ļ�
***************************************************************************************************/

/*****************************************************************************
 ** ��ֲ������ 
 ** ��ֲ"sys.c"��"sys.h" ʱ�����޸�����Ϳ�����
*****************************************************************************/
// USARTx
#define USART_PORT_CLOCK RCC->APB2ENR |= RCC_APB2ENR_USART1EN
#define USART_PORT USART1
// TX
#define USART_TX_GPIO_CLOCK RCC_APB2ENR_GPIOAEN
#define USART_TX_GPIO GPIOA
#define USART_TX_PIN PIN9
// RX
#define USART_RX_GPIO_CLOCK RCC_APB2ENR_GPIOAEN
#define USART_RX_GPIO GPIOA
#define USART_RX_PIN PIN10
// �ж�
//end ��ֲ *****************************************************************

//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_OS 1 //����ϵͳ�ļ����Ƿ�֧��UCOS

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) //0x40011E08

//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //����

/* �������Ͷ�д, ��stdint.h�Ļ����Ͻ���*/
// �з���                                // stdint.h �ϵ����Ͷ���
typedef int8_t s8;   // typedef   signed          char int8_t;
typedef int16_t s16; // typedef   signed short     int int16_t;
typedef int32_t s32; // typedef   signed           int int32_t;
typedef int64_t s64; // typedef   signed       __INT64 int64_t;
// �޷���
typedef uint8_t u8;   // typedef unsigned          char uint8_t;
typedef uint16_t u16; // typedef unsigned short     int uint16_t;
typedef uint32_t u32; // typedef unsigned           int uint32_t;
typedef uint64_t u64; // typedef unsigned       __INT64 uint64_t;
//
typedef volatile uint8_t vu8; // volatile ����Ҫ��ȷ��ʾ
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

//#define  bool      _Bool
//#define  true      1
//#define  false     0

// ��������ƫ��ֵ�� This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET 0x0

// ʱ��ʹ��_�궨��_Ϊ����F4xx�ĸ�ʽ ***********************
#define RCC_APB2ENR_GPIOAEN ((uint32_t)0x00000004) /*!< I/O port A clock enable */
#define RCC_APB2ENR_GPIOBEN ((uint32_t)0x00000008) /*!< I/O port B clock enable */
#define RCC_APB2ENR_GPIOCEN ((uint32_t)0x00000010) /*!< I/O port C clock enable */
#define RCC_APB2ENR_GPIODEN ((uint32_t)0x00000020) /*!< I/O port D clock enable */
#define RCC_APB2ENR_GPIOEEN ((uint32_t)0x00000040) /*!< I/O port D clock enable */
// PIN_�궨��_Ϊ����F4xx�ĸ�ʽ *****************************
#define PIN0 ((uint16_t)0x0001)    /*!< Pin 0 selected */
#define PIN1 ((uint16_t)0x0002)    /*!< Pin 1 selected */
#define PIN2 ((uint16_t)0x0004)    /*!< Pin 2 selected */
#define PIN3 ((uint16_t)0x0008)    /*!< Pin 3 selected */
#define PIN4 ((uint16_t)0x0010)    /*!< Pin 4 selected */
#define PIN5 ((uint16_t)0x0020)    /*!< Pin 5 selected */
#define PIN6 ((uint16_t)0x0040)    /*!< Pin 6 selected */
#define PIN7 ((uint16_t)0x0080)    /*!< Pin 7 selected */
#define PIN8 ((uint16_t)0x0100)    /*!< Pin 8 selected */
#define PIN9 ((uint16_t)0x0200)    /*!< Pin 9 selected */
#define PIN10 ((uint16_t)0x0400)   /*!< Pin 10 selected */
#define PIN11 ((uint16_t)0x0800)   /*!< Pin 11 selected */
#define PIN12 ((uint16_t)0x1000)   /*!< Pin 12 selected */
#define PIN13 ((uint16_t)0x2000)   /*!< Pin 13 selected */
#define PIN14 ((uint16_t)0x4000)   /*!< Pin 14 selected */
#define PIN15 ((uint16_t)0x8000)   /*!< Pin 15 selected */
#define PIN_All ((uint16_t)0xFFFF) /*!< All pins selected */

// GPIOSet ����ר�ú궨�����*****************
#define G_MODE_AIN 0 // ģ������ģʽ
#define G_MODE_IN 1  // ��ͨ����ģʽ
#define G_MODE_OUT 2 // ��ͨ���ģʽ
#define G_MODE_AF 3  // AF����ģʽ

#define G_OTYPE_PP 0 // �������
#define G_OTYPE_OD 1 // ��©���

#define G_OSPEED_10M 1 // GPIO�ٶ�2Mhz
#define G_OSPEED_2M 2  // GPIO�ٶ�2Mhz
#define G_OSPEED_50M 3 // GPIO�ٶ�50Mhz

#define G_PUPD_NOPULL 0 // ����������
#define G_PUPD_UP 1     // ����
#define G_PUPD_DOWN 2   // ����

// ExtiSet �������� ר�� ****************************
#define EXTI_FTIR 1 // �½��ش���
#define EXTI_RTIR 2 // �����ش���

// MCO
#define MCO_SYSCLK 4 // 0100
#define MCO_HSI 5    // 0101
#define MCO_HSE 6    // 0110
#define MCO_NULL 0   // 0000���ر�

/*****************************************************************************
 ** ����  ȫ�ֺ���
 ** ������9��
 ** ħŮ������ ����QȺ�ļ���262901124  ��������
****************************************************************************/
// ��1����_ϵͳ��ʼ��
void System_Init(void); // ��ȡʱ��Ƶ�ʡ���ʼ��SysTick����ʼ��USART
// ��2����_SysTick
//void delay_ms(u32);               // ������ʱ
//void delay_us(u32);               // ΢����ʱ
u64 System_GetTimeMs(void);       // ��ȡ SysTick ��ʱ��, ��λ:ms
u32 System_GetTimeInterval(void); // �������ʱ��
void __INTERVAL(void);            // ��ӡ�������ʱ�䣬 0�����ü�ʱ�㣬 1����ӡ�������ʱ��
// ��3����_����3����
void GPIOSet(GPIO_TypeDef *GPIOx, u32 PINx, u8 MODE, u8 OTYPE, u8 OSPEED, u8 PUPD);
void NVICSet(u8 NVIC_Channel, u8 Preemption);         // ���ȼ�����, �ѷֺ���, 4λ��ռ��, 16��, ���Ӽ�
void EXTISet(GPIO_TypeDef *GPIOx, u16 PINx, u8 TRIM); // �ⲿ�ж����ú���
// ��4����_�����ô���

// ��5����_����
u32 System_GetClock(void); // ��ȡϵͳʱ��Ƶ��
void System_Reset(void);   // ϵͳ��λ
void MCO1Init(uint32_t);   // ��ѡ����,ǰ� RCC_MCO1Source_ +++ HSI , LSE , HSE , PLLCLK
void MCO2Init(uint32_t);   // ��ѡ����,ǰ� RCC_MCO2Source_ +++ SYSCLK , PLLI2SCLK , HSE , PLLCLK

//����Ϊ��ຯ��
void WFI_SET(void);      //ִ��WFIָ��
void INTX_DISABLE(void); //�ر������ж�
void INTX_ENABLE(void);  //���������ж�
void MSR_MSP(u32 addr);  //���ö�ջ��ַ

#endif
