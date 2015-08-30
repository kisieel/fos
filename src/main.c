#include "main.h"

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
	uint8_t buf[20];
	uint32_t i;
//	const char* key = "sampleEncryptKey";

	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIO_config(0x0A, 0, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	GPIOA->BSRRL |= GPIO_BSRR_BS_0;               // Output 1

	SYS_TICK_init();
	KEY_init();
	MENU_init();
	USART_init();
	HALL_init(); // Honeywell	recommends	allowing	10	µs	for	output	voltage	to	stabilize	after	supply	voltage	has	reached	its	final	rated	value.

#ifdef USART_debug
	USART_send("Peripherals initialized.\n");
#endif
	
//	RFM69W_init();

#ifdef USART_debug
	USART_send("External devices initialized.\n");
#endif

	for(;;) {
		_actual->menu_fun(GetKeys());
	}
}
