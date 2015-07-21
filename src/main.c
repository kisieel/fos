#include "stm32l1xx.h"
#include "RFM69W.h"
#include "USART.h"
#include "EEPROM.h"
#include "KEY.h"
#include "MENU.h"

volatile unsigned int PSC = 1;

void SYS_TICK_init(void);

/*
PA0  -
PA1  - KEY_1 button
PA2  - KEY_2 button
PA3  -
PA4  -
PA5  -
PA6  -
PA7  -

PA8  -
PA9  - RFM69 reset pin
PA10 - RFM69 interrupt flag
PA11 -
PA12 -
PA13 -
PA14 -
PA15 - RFM69 SPI CS
*/

/*
PB0  -
PB1  -
PB2  -
PB3  - RFM69 SPI SCK
PB4  - RFM69 SPI MISO
PB5  - RFM69 SPI MOSI
PB6  -
PB7  -

PB8  -
PB9  -
PB10 -
PB11 -
PB12 -
PB13 -
PB14 -
PB15 -
*/

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

void SysTick_Handler(void)
{
	
}

void SYS_TICK_init(void)
{                                                              
	SysTick->CTRL |= SysTick_CTRL_TICKINT;
	SysTick->LOAD = 9000000/PSC;
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	NVIC_SetPriority(SysTick_IRQn, 1);
}