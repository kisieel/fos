#include "stm32l1xx.h"
#include "main.h"

#define MODE_DIFFERENCE    1
#define MODE_CAPTURE       0

volatile uint32_t _milis;
volatile uint8_t _cnt;

void SysTick_Handler(void)
{
	_milis++;
	_cnt++;
	if (_cnt == 10) {
		KeybProc();
		_cnt = 0;
	}
}

void SYS_TICK_init(void)
{                                                              
	SysTick->CTRL |= SysTick_CTRL_TICKINT;
	SysTick->LOAD = 4500000/1000;
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	NVIC_SetPriority(SysTick_IRQn, 1);
}

uint32_t SYS_TICK_timeOut(uint8_t MODE, uint32_t TIMEOUT)
{
	if (MODE == MODE_CAPTURE)
		return _milis;
	else {
		if (MODE == MODE_DIFFERENCE) {
			if (_milis >= TIMEOUT)
				return (_milis - TIMEOUT);
			else
				return (_milis + (0xFFFFFFFF - TIMEOUT));
		}
	}
}