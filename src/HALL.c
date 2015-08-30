#include "main.h"

#define HALL_Timeout        1000
#define HALL_SequenceNumber 5

volatile HALL HALL_Data;

void HALL_init(void)
{
	GPIO_config(0x0A, 6, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PA;     // A block for EXTI6
	EXTI->IMR |= EXTI_IMR_MR6;                        // Input line 13 selection (unmasking)
	EXTI->FTSR |= EXTI_FTSR_TR6;                      // Falling edge selection
	NVIC_SetPriority(EXTI9_5_IRQn, 1);                // Priority set to 0
	NVIC_EnableIRQ(EXTI9_5_IRQn);                     // Interrupt enable
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & EXTI_PR_PR6) {
		if (HALL_Data.SequenceCnt) {
			if (SYS_TICK_timeOut(1, HALL_Data.Time) < HALL_Timeout) {
				HALL_Data.SequenceCnt++;
				if (HALL_Data.SequenceCnt > HALL_SequenceNumber) {
					// Branie
					HALL_Data.SequenceCnt = HALL_SequenceNumber;

				}
				HALL_Data.Time = SYS_TICK_timeOut(0, 0);
			} else {
				HALL_Data.SequenceCnt = 0;
			}
		} else {
			HALL_Data.SequenceCnt++;
			HALL_Data.Time = SYS_TICK_timeOut(0, 0);
		}
		
#ifdef USART_debug
		if (HALL_Data.SequenceCnt == HALL_SequenceNumber) {
			USART_send("Fish caught!\n");
		} else {
			USART_send("Roller moved with #");
			USART_write_buf(HALL_Data.SequenceCnt, DEC);
			USART_send(" sequence.\n");
		}
#endif
		
		EXTI->PR |= EXTI_PR_PR6;
	}
}