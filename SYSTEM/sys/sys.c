#include "sys.h"

/*******************************************************************
  * @file     SYS.c
  * @author   2020年04月21日  
  *
  * @briefs   文件分成两部分
  *           一、常用的三大函数：GPIOSet、NVICSet、EXTISet
  *           二、系统中断分组设置化
  *
  * @note     使用方法: 把 "sys.c"、"sys.h" 两个文件复制到工程文件夹，然后在工程中引用即可
              本文件在原子哥的sys文件基础上改进编写，可升级替换工程中旧的sys文件
              如有更新，将上传到Q群文件夹：262901124
*******************************************************************************/





/*****************************************************************************
 ** 本地变量声明
 *****************************************************************************/
volatile u64 sysTickCnt = 0; // 运行时长，单位：ms

struct // 状态标志结构体,  0=失败/没有, 1=成功/正常
{
    u8 allOK;    // 全部状态正常        0=失败   1=成功
    u8 PrintfOK; // 标记USART是否配置   0=失败   1=成功   防止在配置前调用printf卡死
} xSys;




/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<     第 1 部分     >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<                   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<    常用三大函数    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

/******************************************************************************
 * 函  数： GPIOSet
 * 功  能： 使能相应时钟、配置引脚
 *          外设时钟使能，必须在GPIO配置前，否则会出现问题
 * 参  数：
 * 返回值： 
******************************************************************************/
void GPIOSet(GPIO_TypeDef *GPIOx, u32 allPin, u8 mode, u8 otype, u8 ospeed, u8 pupd)
{
    u32 reg = 0;
    u32 nowPin = 0;

    if (GPIOx == GPIOA)
        RCC->APB2ENR |= RCC_APB2ENR_GPIOAEN;
    if (GPIOx == GPIOB)
        RCC->APB2ENR |= RCC_APB2ENR_GPIOBEN;
    if (GPIOx == GPIOC)
        RCC->APB2ENR |= RCC_APB2ENR_GPIOCEN;
    if (GPIOx == GPIOD)
        RCC->APB2ENR |= RCC_APB2ENR_GPIODEN;
    if (GPIOx == GPIOE)
        RCC->APB2ENR |= RCC_APB2ENR_GPIOEEN;

    // 模拟输入
    if (mode == G_MODE_AIN)
    {
        reg |= 0;
    }
    // 普通输入
    if (mode == G_MODE_IN)
    {
        if (pupd == 0)
            reg |= 0x01 << 2;
        else
            reg |= 0x02 << 2;
    }

    if ((ospeed & 0x03) == 0)
        ospeed = 0x03; // 输出速度，
    // 普通输出
    if (mode == G_MODE_OUT)
    {
        reg = ospeed & 0x03;        // 引脚速度
        reg |= (otype & 0x01) << 2; // 普通推挽、开漏
    }
    // 复用输出
    if (mode == G_MODE_AF)
    {
        reg = ospeed & 0x03;                 // 引脚速度
        reg |= ((otype | 0x02) & 0x03) << 2; // 复用推挽、开漏
    }

    // CHL, pin 0~7
    for (int i = 0; i < 8; i++)
    {
        nowPin = (u32)0x01 << i; // 当前要判断的引脚号
        if ((allPin & nowPin) != 0)
        {                                     // 当前引脚要配置
            GPIOx->CRL &= ~(0x0F << (i * 4)); // 清0
            GPIOx->CRL |= reg << (i * 4);     // 写入新配置
        }
    }

    // CRH, pin 8~15
    for (int i = 0; i < 8; i++)
    {
        nowPin = (u32)0x01 << (i + 8); // 当前要判断的引脚号
        if ((allPin & nowPin) != 0)
        {                                     // 当前引脚要配置
            GPIOx->CRH &= ~(0x0F << (i * 4)); // 清0
            GPIOx->CRH |= reg << (i * 4);     // 写入新配置
        }
    }

    if (pupd == G_PUPD_UP)
        GPIOx->BSRR |= allPin;
    if (pupd == G_PUPD_DOWN)
        GPIOx->BSRR |= allPin << 16;
}

/******************************************************************************
 * 函  数： NVICSet
 * 功  能： 优先级设置，为方便管理及使用FreeRTOS，统一使用4位抢占级(16级),0位子优先级(0级)
 *         直接调用即可，不用提前配置
 * 参  数： 
 * 返回值： 
 * 备  注： 魔女开发板团队 2020年04月21日
******************************************************************************/
void NVICSet(u8 NVIC_Channel, u8 Preemption)
{
    static u8 setGrouped = 0;
    if (setGrouped == 0)
    {
        // 全局分级设置,统一为组4, 值0b11,即：NVIC->IPx中高4位:主级4位(16级), 子级0位(0级）
        SCB->AIRCR = ((u32)0x05FA0000) | (0x03 << 8); // 优先级分组设置, 已查,是3， F103和F429寄存器通用
        setGrouped = 1;
    }

    // 通道中断优先级设置
    NVIC->IP[NVIC_Channel] &= ~(0xF << 4);            // 清空
    NVIC->IP[NVIC_Channel] = (Preemption & 0xF) << 4; // 写入抢占级\优先级

    // 通道中断使能
    NVIC->ISER[NVIC_Channel / 32] |= 1 << (NVIC_Channel % 32); // 使能中断通道

    // 中断失能, 很少用到，这位置占个坑
    //NVIC->ICER[];
}

/***************************************************************************** 
 * 函  数： System_EXTISet
 * 功  能： 外部中断配置函数
 *         重要: 一次只能配置1个IO口,  2020-2-26
 *         只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
 *         该函数会自动开启对应中断,以及屏蔽线  
 *         
 * 参  数： 【GPIOx】:GPIOA~G, 代表GPIOA~G
 *          【BITx】:PIN0~15, 需要使能的位;
 *          【TRIM】:触发模式, EXTI_FTIR/1:下降沿;  EXTI_RTIR/2:上升沿; 3:任意电平触发
 * 返  回： 
*****************************************************************************/
void EXTISet(GPIO_TypeDef *GPIOx, u16 PINx, u8 TRIM)
{
    u8 gpioNum = 0;
    u8 pinNum = 0;

    // 转换GPIOx为数字
    if (GPIOx == GPIOA)
        gpioNum = 0;
    if (GPIOx == GPIOB)
        gpioNum = 1;
    if (GPIOx == GPIOC)
        gpioNum = 2;
    if (GPIOx == GPIOD)
        gpioNum = 3;
    if (GPIOx == GPIOE)
        gpioNum = 4;
    if (GPIOx == GPIOF)
        gpioNum = 5;
    if (GPIOx == GPIOG)
        gpioNum = 6;

    // 转换PINx为数字
    for (int i = 0; i < 16; i++)
    {
        if (PINx == ((u32)1 << i))
        {
            pinNum = i;
            break;
        }
    }

    u8 offSet = (pinNum % 4) * 4;                    // 寄存器内偏移
    RCC->APB2ENR |= 0x01;                            // 使能io复用时钟
    AFIO->EXTICR[pinNum / 4] &= ~(0x000F << offSet); // 清0
    AFIO->EXTICR[pinNum / 4] |= gpioNum << offSet;   // EXTI.BITx映射到GPIOx.BITx

    EXTI->IMR |= PINx; // 使能line BITx上的中断, 1:使能  0:屏蔽
                       //EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
    if (TRIM & 0x01)
        EXTI->FTSR |= PINx; //line BITx上事件下降沿触发
    if (TRIM & 0x02)
        EXTI->RTSR |= PINx; //line BITx上事件上升降沿触发
}

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI
void WFI_SET(void)
{
    __ASM volatile("wfi");
}
//关闭所有中断
void INTX_DISABLE(void)
{
    __ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
    __ASM volatile("cpsie i");
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr)
{
    MSR MSP, r0 //set Main Stack value
                 BX r14
}

// 进入待机模式
void System_Standby(void)
{
    SCB->SCR |= 1 << 2;      //使能SLEEPDEEP位 (SYS->CTRL)
    RCC->APB1ENR |= 1 << 28; //使能电源时钟
    PWR->CSR |= 1 << 8;      //设置WKUP用于唤醒
    PWR->CR |= 1 << 2;       //清除Wake-up 标志
    PWR->CR |= 1 << 1;       //PDDS置位
    WFI_SET();               //执行WFI指令
}

// 系统软复位
void System_Reset(void)
{
    SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}

// 把时钟输出引脚, 方便使用示波器检查时钟是否正确
void System_MCO1Init(u8 source)
{
    RCC->CFGR &= ~(0x0f << 24);
    RCC->CFGR |= (source & 0x0f) << 24;
}
