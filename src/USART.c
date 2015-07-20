#include "stm32l1xx.h"
#include "USART.h"

void USART_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                  // Clock for GPIOB 
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;               // Clock for USART1
	
	GPIOB->MODER |= GPIO_MODER_MODER6_1                 // Alternate function mode
	              | GPIO_MODER_MODER7_1;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_ODR_6                // Push-pull
	                 | GPIO_OTYPER_ODR_7);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6          // Very low speed 400 kHz
	                  | GPIO_OSPEEDER_OSPEEDR7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6                 // No pull-up/pull down
	                | GPIO_PUPDR_PUPDR7);
	GPIOB->AFR[0] |= (GPIO_AFRL_AFRL6 & (7 << 4*6))     // AF7- USART1 TX/RX
	               | (GPIO_AFRL_AFRL7 & (7 << 4*7));
	
	USART1->BRR = (SystemCoreClock / 19200);            // Baudrate = (sysclock/baudrate)
	
	USART1->CR1 |= USART_CR1_UE                         // USART enable
	             | USART_CR1_TE                         // Transmit enable
	             | USART_CR1_RE;
}

void USART_putchar(char ch)
{
	while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = ch;
}

void USART_send(char * text)
{
	while (*text!='\0')
	{
		USART_putchar(*text);
		text++;
	}
}