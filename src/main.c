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
	int i;
	uint8_t buf[10];
//	const char* key = "sampleEncryptKey";

	// Keep power supply
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;            // Clock for GPIOA
	GPIO_config(0x0A, 0, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	GPIOA->BSRRL |= GPIO_BSRR_BS_0;               // Output 1
	
//	_KEY_init();
//	_MENU_init();
	_USART_init();
	_RFM69W_init();
	
//	_RFM69W_send(RFM69W_write, REG_AESKEY1, key[0]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY2, key[1]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY3, key[2]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY4, key[3]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY5, key[4]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY6, key[5]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY7, key[6]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY8, key[7]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY9, key[8]);
//	_RFM69W_send(RFM69W_write, REG_AESKEY10, key[9]);
//	
//	if (key != 0) {
//		for (i = 0; i < 16; i++)
//			_RFM69W_send(RFM69W_write, REG_AESKEY1 + i, key[i]);
//  }
	
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	
//	for (i = 1000000; i>1; i--);
//	
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
	
//		_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//		_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//		_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	_RFM69W_send(RFM69W_write, 0x18, 0x00);
//	SYS_TICK_init();
//	_dbl2stri(buf, _RFM69W_send_poll(RFM69W_read, 0x18, 0x00), 0);
//	_USART_send(buf);
//	_dbl2stri(buf, _RFM69W_send_poll(RFM69W_read, 0x18, 0x00), 0);
//	_USART_send(buf);
//	_dbl2stri(buf, _RFM69W_send_poll(RFM69W_read, 0x18, 0x00), 0);
//	_USART_send(buf);
//	_dbl2stri(buf, _RFM69W_send_poll(RFM69W_read, 0x18, 0x00), 0);
//	_USART_send(buf);
	
	for(;;) {
		
	}
}

void SysTick_Handler(void)
{
}

void SYS_TICK_init(void)
{                                                              
	SysTick->CTRL |= SysTick_CTRL_TICKINT;
	SysTick->LOAD = 9000000/4;
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	NVIC_SetPriority(SysTick_IRQn, 1);
}
