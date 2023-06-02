/*
encoding: windows 1251
*/
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

int x;
uint16_t datamod;

void delay(uint32_t i)// пауза
{
while(i--);
}


void port()
{  //настройка портов
  RCC->APB2ENR |= RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB; 
  GPIO_InitTypeDef gpiob;
  gpiob.GPIO_Speed = GPIO_Speed_10MHz;
  gpiob.GPIO_Mode = GPIO_Mode_Out_PP;
  gpiob.GPIO_Pin= GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_Init(GPIOB , &gpiob);
}


void initUSART() 
{
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart_init;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  
  gpio.GPIO_Speed = GPIO_Speed_50MHz;


  //tx pa9
  gpio.GPIO_Mode=GPIO_Mode_AF_PP;
  gpio.GPIO_Pin= GPIO_Pin_9;
  GPIO_Init(GPIOA , &gpio);
  
  //rx pa10
  gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  gpio.GPIO_Pin= GPIO_Pin_10;
  GPIO_Init(GPIOA , &gpio);
  
  usart_init.USART_BaudRate = 9600;
  usart_init.USART_WordLength= USART_WordLength_8b;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &usart_init);
  USART1->CR1|=USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
  USART_ClearFlag (USART1, USART_FLAG_CTS | USART_FLAG_LBD | USART_FLAG_TC | USART_FLAG_RXNE);
  
  USART_Cmd(USART1, ENABLE);
  
  
}


void sendOne()
{
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_SET);
  delay(240);
  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_SET); 
  delay(240);
  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET);
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
  delay(240);
}


void sendNull()
{
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
  delay(240);
  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_SET);  
  delay(240);
  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET);
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
  delay(240);
}

/// @brief Отправка регистру
/// @param prov 
void sendRegister(uint16_t prov)
{
  delay(720);
  int i = 7;
  
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_RESET);
  
  sendNull();
  sendNull();
  
  
  do
  {
    if (prov & (1<<i))
      sendOne();
    else
      sendNull();
    
    i = i-1;
    
  } 
  while (i != 0);
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET);
  
}


void USART1_IRQHandler(void)
{
  while((USART1->SR & USART_SR_RXNE)==0){}

  uint8_t data =USART1->DR;
  datamod = data;
  sendRegister(datamod);
  
  while((USART1->SR & USART_SR_TXE)==0){}
  USART1->DR=data;
  
}


int main(void)
{
  port();
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET);
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET);
  initUSART();
  NVIC_EnableIRQ(USART1_IRQn);
  while (1)
  {
    /* code */
  }
  
}
  





