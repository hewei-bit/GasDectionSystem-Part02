#ifndef __LORA_H
#define __LORA_H

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define   LORA_SET_ADDR            "AT+ADDR=FF,FF\r\n" 
       

#define   LORA_USARTx              USART3                       
#define   LORA_USART_CLOCK_CMD     RCC->APB1ENR |= (u32)1<<18     
#define   LORA_USART_AFx           G_AF_USART2                    

#define   LORA_USART_CLK           45                             
#define   LORA_USART_BRR           115200                         

#define   LORA_TX_GPIOx            GPIOB                         
#define   LORA_TX_PINx             PIN10                            

#define   LORA_RX_GPIOx            GPIOB                          
#define   LORA_RX_PINx             PIN11 

#define   LORA_AUX_GPIOx           GPIOC                          
#define   LORA_AUX_PINx            PIN8

#define   LORA_MD0_GPIOx           GPIOC                          
#define   LORA_MD0_PINx            PIN7                                 

#define   LORA_USART_IRQHANDLER    USART3_IRQHandler             
#define   LORA_USART_IRQN          USART3_IRQn                   





void Lora_Init(void);
void Lora_Set(void);

void Lora_SendChar(char);
void Lora_SendString(char *);         
void Lora_SendIntegerToString(u32);
 
void Lora_ReceiveFlash(void);




#endif 

