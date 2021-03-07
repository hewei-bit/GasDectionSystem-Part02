#include "sys.h"
#include "delay.h"
#include "usart3.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif

//串口接收缓存区
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; //接收缓冲,最大USART3_MAX_RECV_LEN个字节.
u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; //发送缓冲,最大USART3_MAX_SEND_LEN字节

static USART_InitTypeDef USART_InitStructure;

//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 USART3_RX_STA = 0;

//初始化IO 串口3
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率
void usart3_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //串口3时钟使能

    USART_DeInit(USART3); //复位串口3

    //USART3_TX   PB10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //初始化PB10

    //USART3_RX	  PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);                //初始化PB11

    USART_InitStructure.USART_BaudRate = bound;                                     //波特率一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式

    USART_Init(USART3, &USART_InitStructure);      //初始化串口	3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //开启中断
    USART_Cmd(USART3, ENABLE);                     //使能串口

    //设置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           //根据指定的参数初始化VIC寄存器

    //TIM7_Int_Init(99,7199);		//10ms中断
    USART3_RX_STA = 0; //清零
                       //TIM_Cmd(TIM7,DISABLE);			//关闭定时器7
}

//接收LORA通信模块
void USART3_IRQHandler(void)
{
    u8 res;
    OS_ERR err;

    int len = 0;
    int t = 0;

#ifdef SYSTEM_SUPPORT_OS
    //进入中断
    OSIntEnter();
#endif

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        res = USART_ReceiveData(USART3);
        if ((USART3_RX_STA & 0x8000) == 0) //接受未完成
        {
            if (USART3_RX_STA & 0x4000) //接收到了0x0d
            {
                if (res != 0x0a)
                    USART3_RX_STA = 0; //接受错误,重新开始
                else
                    USART3_RX_STA |= 0x8000; //接收完成了
            }
            else //还没收到0X0D
            {
                if (res == 0x0d)
                    USART3_RX_STA |= 0x4000;
                else
                {
                    USART3_RX_BUF[USART3_RX_STA & 0X3FFF] = res;
                    USART3_RX_STA++;
                    if (USART3_RX_STA > (USART3_MAX_RECV_LEN - 1))
                        USART3_RX_STA = 0; //接收数据错误,重新开始接收
                }
            }
        }
    }

    if (USART3_RX_STA & 0x8000)
    {
        len = USART3_RX_STA & 0x3FFF; //得到此次接收数据的长度
        //		for(t=0;t<len;t++)
        //		{
        //			USART_SendData(USART1, USART3_RX_BUF[t]);//向串口1发送数据
        //			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
        //		}
        //		printf("LORA_RX:%s\r\n",USART3_RX_BUF);

#if 1 //发送消息队列
        OSQPost((OS_Q *)&g_queue_usart3,
                (void *)USART3_RX_BUF,
                (OS_MSG_SIZE)len,
                (OS_OPT)OS_OPT_POST_FIFO,
                (OS_ERR *)&err);
        if (err != OS_ERR_NONE)
        {
            printf("[USART3_IRQHandler]OSQPost error code %d\r\n", err);
        }
#endif
    }

#ifdef SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

//串口3,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u3_printf(char *fmt, ...)
{
    u16 i, j;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)USART3_TX_BUF, fmt, ap);
    va_end(ap);
    i = strlen((const char *)USART3_TX_BUF); //此次发送数据的长度
    for (j = 0; j < i; j++)                  //循环发送数据
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
            ; //循环发送,直到发送完毕
        USART_SendData(USART3, USART3_TX_BUF[j]);
    }
}

void usart3_send_str(char *pstr)
{
    char *p = pstr;

    while (p && *p != '\0')
    {
        //发送数据
        USART_SendData(USART3, *p);
        //等待发送完毕
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
            ;
        p++;
    }
}

//串口3波特率和校验位配置
//bps:波特率（1200~115200）
//parity:校验位（无、偶、奇）
// void usart3_set(u8 bps,u8 parity)
// {
//    static u32 bound=0;

// 	switch(bps)
// 	{
// 		case LORA_TTLBPS_1200:   bound=1200;     break;
// 		case LORA_TTLBPS_2400:   bound=2400;     break;
// 		case LORA_TTLBPS_4800:   bound=4800;     break;
// 		case LORA_TTLBPS_9600:   bound=9600;     break;
// 		case LORA_TTLBPS_19200:  bound=19200;    break;
// 		case LORA_TTLBPS_38400:  bound=38400;    break;
// 		case LORA_TTLBPS_57600:  bound=57600;    break;
// 		case LORA_TTLBPS_115200: bound=115200;   break;
// 	}

// 	USART_Cmd(USART3, DISABLE); //关闭串口
// 	USART_InitStructure.USART_BaudRate = bound;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;

// 	if(parity==LORA_TTLPAR_8N1)//无校验
// 	{
// 		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 		USART_InitStructure.USART_Parity = USART_Parity_No;
// 	}
// 	else if(parity==LORA_TTLPAR_8E1)//偶校验
// 	{
// 		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
// 		USART_InitStructure.USART_Parity = USART_Parity_Even;
// 	}
// 	else if(parity==LORA_TTLPAR_8O1)//奇校验
// 	{
// 		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
// 		USART_InitStructure.USART_Parity = USART_Parity_Odd;
// 	}
// 	USART_Init(USART3, &USART_InitStructure); //初始化串口3
//    USART_Cmd(USART3, ENABLE); //使能串口

// }

//串口接收使能控制
//enable:0,关闭 1,打开
void usart3_rx(u8 enable)
{
    USART_Cmd(USART3, DISABLE); //失能串口

    if (enable)
    {
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    }
    else
    {
        USART_InitStructure.USART_Mode = USART_Mode_Tx; //只发送
    }

    USART_Init(USART3, &USART_InitStructure); //初始化串口3
    USART_Cmd(USART3, ENABLE);                //使能串口
}
