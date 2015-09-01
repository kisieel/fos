#include "main.h"

/*
SysTick    - System timer
SPI1       - RFM69
EXTI_10_15 - RFM69 EXTI13
EXTI_9_5   - HALL EXTI6
USART1     - USART
TIM2       - LED
TIM10      - LED
DMA1_Ch2   - LED
TIM3       - BUZZER
TIM9       - BUZZER
DAC1       - BUZZER
*/

/*
V 0.2 china
PA0  - MAIN_BUTTON
PA1  - KEY_1 button
PA2  - KEY_2 button
PA4  - BUZ_volume
PA5  - VIN ADC
PA6  - HALL
PA7  - 5V_enable

PA8  - RFM69 DIO5
PA9  - RFM69 reset pin
PA13 - JTMS
PA14 - JTCK
PA15 - RFM69 SPI CS

PB3  - RFM69 SPI SCK
PB4  - RFM69 SPI MISO
PB5  - RFM69 SPI MOSI
PB6  - UART_TX
PB7  - UART_RX

PB10 - RGB_LED
PB11 - RFM69 DIO1
PB12 - RFM69 DIO2
PB13 - BUZ_tone
PB14 - RFM69 DIO3
PB15 - RFM69 DIO4
PC13 - RFM69 DIO0 Wake uC
*/

int main()
{	
	uint8_t buf[20];
//	uint32_t i;
	uint8_t i,j = 0;
//	const char* key = "sampleEncryptKey";

	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIO_config(0x0A, 0, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	GPIOA->BSRRL |= GPIO_BSRR_BS_0;               // Output 1

	SYS_TICK_init();
	KEY_init();
	MENU_init();
//	USART_init();
//	HALL_init(); // Honeywell	recommends	allowing	10	µs	for	output	voltage	to	stabilize	after	supply	voltage	has	reached	its	final	rated	value.

#ifdef USART_debug
	USART_send("Peripherals initialized.\n");
#endif
	
//	RFM69W_init();
	_BUZZER_init();
	
//	_LED_init();
//	_LED_off();
//	animate_mode[0] = animate_mode_1;
	
#ifdef USART_debug
	USART_send("External devices initialized.\n");
#endif

//	for(i=0; i<10;i++)
//	{
//		music.samples[0].tone[i] = temp_tone[i];
//		music.samples[0].tempo[i] = temp_tempo[i];
//	}
//	music.samples[0].position = 0;
//	
//	_BUZZER_play_music(0);

	for(;;) {
		_actual->menu_fun(GetKeys());
//		i = SYS_TICK_timeOut(0,0);
//		_LED_on();
//		_LED_animate();
//		while (SYS_TICK_timeOut(1, i) < 1000);
		
	}
}
