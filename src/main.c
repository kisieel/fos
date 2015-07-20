#include "stm32l1xx.h"
#include "RFM69W.h"
#include "USART.h"

int main()
{
	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIOA->MODER |= GPIO_MODER_MODER0_0;          // GP output mode
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);  // Very low speed 400 kHz
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);         // Push-pull, no pull-up/pull down
	GPIOA->BSRRL |= GPIO_BSRR_BS_0;               // Output 1
	
	// USART1_init: PB6/7:TX/RX, 19.2 kbps, 8 bit, no parity, 1 stop bit
	USART_init();
	
	//RFM69W_init: 
	RFM69W_init();
	
	for(;;) {
		
	}
}