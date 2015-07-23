#include "stm32l1xx.h"
#include "RFM69W.h"
#include "USART.h"
#include "EEPROM.h"
#include "KEY.h"
#include "MENU.h"
#include "FLOAT.h"

void SYS_TICK_init(void);

/*
V  0.2
PA0  - MAIN_BUTTON
PA1  - KEY_1 button
PA2  - KEY_2 button
PA4  - BUZ_volume
PA5  - VIN ADC
PA6  - HALL
PA7  - 5V_enable

PA9  - RFM69 reset pin
PA10 - RFM69 interrupt flag
PA13 - JTMS
PA14 - JTCK
PA15 - RFM69 SPI CS

PB3  - RFM69 SPI SCK
PB4  - RFM69 SPI MISO
PB5  - RFM69 SPI MOSI
PB6  - UART_TX
PB7  - UART_RX

PB10 - RGB_LED
PB13 - BUZ_tone
*/

int main()
{
	char buf[100];
	
	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIO_config(0x0A, 0, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	GPIOA->BSRRL |= GPIO_BSRR_BS_0;               // Output 1
	
	// USART1_init: PB6/7:TX/RX, 19.2 kbps, 8 bit, no parity, 1 stop bit
	USART_init();
	
	//RFM69W_init: 
//	RFM69W_init();
	for(;;) {
		
	}
}

void SysTick_Handler(void)
{
	
}

void SYS_TICK_init(void)
{                                                              
	SysTick->CTRL |= SysTick_CTRL_TICKINT;
	SysTick->LOAD = 9000000/1;
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	NVIC_SetPriority(SysTick_IRQn, 1);
}