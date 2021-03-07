#include "lora.h"
 

#define   LORA_SET_WLRATE    "AT+WLRATE=5,5\r\n"    
#define   LORA_SET_UART      "AT+UART=7,0\r\n"       
#define   LORA_SET_WLTIME    "AT+WLTIME=0\r\n"      
#define   LORA_SET_TPOWER    "AT+TPOWER=3\r\n"      
#define   LORA_SET_CWMODE    "AT+CWMODE=0\r\n"      
#define   LORA_SET_TMODE     "AT+TMODE=0\r\n"        
 

u8 _FlagReceiveNewData = 0;
u8 cReceiveLen=0;
char cReceiveData[255];
char cReceiveBuffer[255];
char pcTemp[255];
   
static  void    AUX(u8 u);
static  void    MD0(u8 u);    
   
void AUX(u8 u)
{
    if(u==0)
        LORA_AUX_GPIOx->ODR  &=~LORA_AUX_PINx ;
    else
        LORA_AUX_GPIOx ->ODR |= LORA_AUX_PINx  ;   
} 

  
void MD0(u8 u)
{
    if(u==0)
        LORA_MD0_GPIOx ->ODR &=~LORA_MD0_PINx;
    else
        LORA_MD0_GPIOx->ODR |= LORA_MD0_PINx;      
}


void Debug_ReceiveFlash(void)
{      
    TIM7->CR1 = 0;                            
    
    memcpy( cReceiveData , cReceiveBuffer,45);     
    
    memset (cReceiveBuffer ,0,45);            
    cReceiveLen=0;                                
    
    _FlagReceiveNewData = 1;             
}




void TIM7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN ;    
 	TIM7->ARR     = 20-1;  	             
	TIM7->PSC     = 9000-1;  	         
	TIM7->CNT     = 0;  		        	  
	TIM7->DIER   |= 1<<0;                 
	TIM7->CR1     = 0;                  
    	
    NVICSet(TIM7_IRQn,5); 
}   




void Lora_Init(void)
{
    float T;
    u16 M,F;      

    GPIOSet (LORA_TX_GPIOx, LORA_TX_PINx, G_MODE_AF, G_OTYPE_PP, G_OSPEED_50M , G_PUPD_UP );  
    GPIOSet (LORA_RX_GPIOx, LORA_RX_PINx, G_MODE_AF, G_OTYPE_PP, G_OSPEED_50M , G_PUPD_UP );  
          
    GPIOSet (LORA_MD0_GPIOx , LORA_MD0_PINx , G_MODE_OUT ,G_OTYPE_PP , G_OSPEED_50M , G_PUPD_DOWN);
    GPIOSet (LORA_AUX_GPIOx , LORA_AUX_PINx , G_MODE_IN ,G_OTYPE_PP , G_OSPEED_50M , G_PUPD_DOWN);
    AUX (0);
    MD0 (0);            
    
    LORA_USART_CLOCK_CMD ;            
	T=(float)(LORA_USART_CLK *1000000)/(LORA_USART_BRR *16); 
	M=T;				        	 
	F=(T-M)*16; 	                  
    M<<=4;
	M=M+F;  
    LORA_USARTx ->BRR = M;           
    LORA_USARTx ->CR1  = 0;               
    LORA_USARTx ->CR1 |= 1<<2;                           
    LORA_USARTx ->CR1 |= 1<<3;       
    LORA_USARTx ->CR1 |= 0<<5;        
    LORA_USARTx ->CR1 |= 0<<10;       
    LORA_USARTx ->CR1 |= 0<<12;       
    LORA_USARTx ->CR1 |= 0<<15;                
    LORA_USARTx ->CR1 |= 1<<13;          
         
    Lora_Set();                      
 	NVICSet(LORA_USART_IRQN,3);  
    TIM7_Init();  
    printf("无线数据传输 配置完成/n");   
      
}
                

void Lora_SendChar(char c)
{
    while((LORA_USARTx ->SR  & (1<<6)) == 0); 
    LORA_USARTx ->DR = c;       
}




void Lora_SendString(char *c)
{  
    while(*c != 0 )     
        Lora_SendChar(*c++);        
}  



void Lora_SendIntegerToString(u32 i)
{
    sprintf(pcTemp ,"%d\n",i );
    Lora_SendString(pcTemp);
}    



void Lora_ReceiveFlash(void)
{      
    TIM7->CR1 = 0;                             
    
    memcpy( cReceiveData , cReceiveBuffer,25); 
    
    memset (cReceiveBuffer ,0,25);             
    cReceiveLen=0;                                
    
    _FlagReceiveNewData = 1;     
}


void Lora_Set(void)
{   
    char t = 110; 
                                               
    delay_ms(t);        
    while(  (LORA_AUX_GPIOx ->IDR & LORA_AUX_PINx)!=0);     
    
    MD0(1);                                   
    GPIOSet(LORA_AUX_GPIOx ,LORA_AUX_PINx ,G_MODE_OUT  ,G_OTYPE_PP ,G_OSPEED_50M ,G_PUPD_DOWN );
    AUX(0);     
                 
    delay_ms(t);   
    Lora_SendString("AT+WLRATE=5,5\r\n");    
    
    delay_ms(t);  
    Lora_SendString("AT+UART=7,0\r\n"  );     
    
    delay_ms(t);  
    Lora_SendString("AT+WLTIME=0\r\n" );    
   
    delay_ms(t);  
    Lora_SendString("AT+TPOWER=3\r\n" );      
    
    delay_ms(t);  
    Lora_SendString("AT+CWMODE=0\r\n" );     
    
    delay_ms(t);  
    Lora_SendString("AT+TMODE=0\r\n");        
    
    delay_ms(t);  
    Lora_SendString(LORA_SET_ADDR );          
    
    delay_ms(t);       
    MD0(0);                                  
    GPIOSet(LORA_AUX_GPIOx ,LORA_AUX_PINx ,G_MODE_IN ,G_OTYPE_PP ,G_OSPEED_50M ,G_PUPD_DOWN );   
    delay_ms(t);         
}
