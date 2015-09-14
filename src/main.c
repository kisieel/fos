#include "main.h"

/*
SysTick    - System timer
SPI1       - RFM69
EXTI_10_15 - RFM69 EXTI13
EXTI_9_5   - HALL EXTI6
TIM6       - HALL
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

SystemType System;

int main()
{	
	uint32_t data;
	uint8_t buffer_send[5];
	
	SYS_TICK_init();
	
	data = SYS_TICK_timeOut(0,0);
	while (SYS_TICK_timeOut(1, data) < 400);
	
	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIO_config(0x0A, 3, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	GPIO_config(0x0A, 7, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	Power3VOn;
	Power5VOn;

	data = EEPROM_32_read(EEPROM_ConfAddress1);
	
	System.ActAnimation  = (data & EEPROM_1_ActAnimation) >> EEPROM_1_ActAnimationPosition;
	System.ActColor      = (data & EEPROM_1_ActColor) >> EEPROM_1_ActColorPosition;
	System.ActBrightness = (data & EEPROM_1_ActBrightness) >> EEPROM_1_ActBrightnessPosition;
	System.ActAlarmTone  = (data & EEPROM_1_ActAlarmTone) >> EEPROM_1_ActAlarmTonePosition;
	System.ActAlarmVol   = (data & EEPROM_1_ActAlarmVol) >> EEPROM_1_ActAlarmVolPosition;
	System.ActAlarmTempo = (data & EEPROM_1_ActAlarmTempo) >> EEPROM_1_ActAlarmTempoPosition;
	System.ActMusic      = (data & EEPROM_1_ActMusic) >> EEPROM_1_ActMusicPosition;
	
	KEY_init();
	MENU_init();
	USART_init();
	HALL_init(); // Honeywell	recommends	allowing	10	µs	for	output	voltage	to	stabilize	after	supply	voltage	has	reached	its	final	rated	value.
	
#ifdef USART_debug
	USART_send("Peripherals initialized.\n");
#endif
	
	RFM69W_init();
	_BUZZER_init();
	_LED_init();
	_LED_off();

#ifdef USART_debug
	USART_send("External devices initialized.\n");
#endif

	_LED_set_color_list(3, System.ActColor);
	_LED_change_brightness_limit_list(System.ActBrightness);
	_LED_on();
	_BUZZER_alarm_set_tone_list(System.ActAlarmTone);
	_BUZZER_alarm_set_vol_list(System.ActAlarmVol);
	_BUZZER_alarm_set_tempo_list(System.ActAlarmTempo);
	BUZZER_reset_timer();

#ifdef USART_debug
	USART_send("System values imported from EEPROM.\n\n");
#endif

#ifdef USART_debug
	USART_send("-1- Hunter mode.\n");
#endif

	// Send info to central unit
	buffer_send[0] = MENU_RF_Recruit;
	buffer_send[1] = (data & 0x000000FF) >> 0;
	buffer_send[2] = (data & 0x0000FF00) >> 8;
	buffer_send[3] = (data & 0x00FF0000) >> 16;
	buffer_send[4] = (data & 0xFF000000) >> 24;
	RFM69W_sendWithRetry(0x00, buffer_send, 5, 15, 10);

	// Block device while the power button is still pressed
	while(GPIOA->IDR & GPIO_IDR_IDR_0);

	for(;;) {
		if (HALL_Data.Result)
			_LED_animate(System.ActAnimation);
		else
			_actual->menu_fun(GetKeys());
		
		if (GPIOA->IDR & GPIO_IDR_IDR_0) {
			data = SYS_TICK_timeOut(0, 0);
			while (GPIOA->IDR & GPIO_IDR_IDR_0) {
				if (SYS_TICK_timeOut(1, data) > TurnOffTime) {
					EEPROM_SystemBackup();
					_LED_off();
					buffer_send[0] = MENU_RF_LogOff;
					RFM69W_sendWithRetry(0x00, buffer_send, 1, 15, 10);
#ifdef USART_debug
					USART_send("Power off detected. System backed up. Switching off.\n");
#endif
					Power3VOff;
					Power5VOff;
					// Preventing from further code execution
					for (;;);
				}
			}
		}
	}
}
