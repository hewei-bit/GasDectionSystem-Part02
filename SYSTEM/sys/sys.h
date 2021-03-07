#ifndef __SYS_H
#define __SYS_H
#include "stm32f10x.h"

#include "stdio.h"
#include "string.h"
/************************************************************************************************
  * @file     SYS.h
  * @author   2020年04月21日  魔女开发板 团队
  *
  * @briefs   文件两功能
  *           1：用于修改调试用USARTx的移植配置, 本示例使用的是USART1、PA9\PA10
  *           2：声明一些宏、全局函数, 移植时，这些是不用修改的
  *
  * @note     使用方法: 把 "sys.c"、"sys.h" 两个文件复制到工程文件夹，然后在工程中引用即可
              本文件在原子哥的sys文件基础上改进编写，可升级替换工程中旧的sys文件
***************************************************************************************************/

/*****************************************************************************
 ** 移植参数， 
 ** 移植"sys.c"、"sys.h" 时，仅修改这里就可以了
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
// 中断
//end 移植 *****************************************************************

//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_OS 1 //定义系统文件夹是否支持UCOS

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
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

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //输入

/* 常用类型短写, 在stdint.h的基础上进行*/
// 有符号                                // stdint.h 上的类型定义
typedef int8_t s8;   // typedef   signed          char int8_t;
typedef int16_t s16; // typedef   signed short     int int16_t;
typedef int32_t s32; // typedef   signed           int int32_t;
typedef int64_t s64; // typedef   signed       __INT64 int64_t;
// 无符号
typedef uint8_t u8;   // typedef unsigned          char uint8_t;
typedef uint16_t u16; // typedef unsigned short     int uint16_t;
typedef uint32_t u32; // typedef unsigned           int uint32_t;
typedef uint64_t u64; // typedef unsigned       __INT64 uint64_t;
//
typedef volatile uint8_t vu8; // volatile 作用要明确标示
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

//#define  bool      _Bool
//#define  true      1
//#define  false     0

// 中量向量偏移值， This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET 0x0

// 时钟使能_宏定义_为兼容F4xx的格式 ***********************
#define RCC_APB2ENR_GPIOAEN ((uint32_t)0x00000004) /*!< I/O port A clock enable */
#define RCC_APB2ENR_GPIOBEN ((uint32_t)0x00000008) /*!< I/O port B clock enable */
#define RCC_APB2ENR_GPIOCEN ((uint32_t)0x00000010) /*!< I/O port C clock enable */
#define RCC_APB2ENR_GPIODEN ((uint32_t)0x00000020) /*!< I/O port D clock enable */
#define RCC_APB2ENR_GPIOEEN ((uint32_t)0x00000040) /*!< I/O port D clock enable */
// PIN_宏定义_为兼容F4xx的格式 *****************************
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

// GPIOSet 函数专用宏定义参数*****************
#define G_MODE_AIN 0 // 模拟输入模式
#define G_MODE_IN 1  // 普通输入模式
#define G_MODE_OUT 2 // 普通输出模式
#define G_MODE_AF 3  // AF功能模式

#define G_OTYPE_PP 0 // 推挽输出
#define G_OTYPE_OD 1 // 开漏输出

#define G_OSPEED_10M 1 // GPIO速度2Mhz
#define G_OSPEED_2M 2  // GPIO速度2Mhz
#define G_OSPEED_50M 3 // GPIO速度50Mhz

#define G_PUPD_NOPULL 0 // 不带上下拉
#define G_PUPD_UP 1     // 上拉
#define G_PUPD_DOWN 2   // 下拉

// ExtiSet 函数参数 专用 ****************************
#define EXTI_FTIR 1 // 下降沿触发
#define EXTI_RTIR 2 // 上升沿触发

// MCO
#define MCO_SYSCLK 4 // 0100
#define MCO_HSI 5    // 0101
#define MCO_HSE 6    // 0110
#define MCO_NULL 0   // 0000，关闭

/*****************************************************************************
 ** 声明  全局函数
 ** 数量：9个
 ** 魔女开发板 资料Q群文件：262901124  自由下载
****************************************************************************/
// 第1部分_系统初始化
void System_Init(void); // 获取时钟频率、初始化SysTick、初始化USART
// 第2部分_SysTick
//void delay_ms(u32);               // 毫秒延时
//void delay_us(u32);               // 微秒延时
u64 System_GetTimeMs(void);       // 获取 SysTick 计时数, 单位:ms
u32 System_GetTimeInterval(void); // 监察运行时间
void __INTERVAL(void);            // 打印监察运行时间， 0：设置计时点， 1：打印监察运行时间
// 第3部分_常用3大函数
void GPIOSet(GPIO_TypeDef *GPIOx, u32 PINx, u8 MODE, u8 OTYPE, u8 OSPEED, u8 PUPD);
void NVICSet(u8 NVIC_Channel, u8 Preemption);         // 优先级设置, 已分好组, 4位抢占级, 16级, 无子级
void EXTISet(GPIO_TypeDef *GPIOx, u16 PINx, u8 TRIM); // 外部中断配置函数
// 第4部分_调试用串口

// 第5部分_辅助
u32 System_GetClock(void); // 获取系统时钟频率
void System_Reset(void);   // 系统软复位
void MCO1Init(uint32_t);   // 可选参数,前辍 RCC_MCO1Source_ +++ HSI , LSE , HSE , PLLCLK
void MCO2Init(uint32_t);   // 可选参数,前辍 RCC_MCO2Source_ +++ SYSCLK , PLLI2SCLK , HSE , PLLCLK

//以下为汇编函数
void WFI_SET(void);      //执行WFI指令
void INTX_DISABLE(void); //关闭所有中断
void INTX_ENABLE(void);  //开启所有中断
void MSR_MSP(u32 addr);  //设置堆栈地址

#endif
