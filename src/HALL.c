#include "main.h"

#define HALL_Timeout        1000
#define HALL_SequenceNumber 4

volatile HALL HALL_Data;

void HALL_init(void)
{
	GPIO_config(0x0A, 6, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PA;     // A block for EXTI6
	EXTI->IMR |= EXTI_IMR_MR6;                        // Input line 13 selection (unmasking)
	EXTI->FTSR |= EXTI_FTSR_TR6;                      // Falling edge selection
	NVIC_SetPriority(EXTI9_5_IRQn, 2);                // Priority set
	NVIC_EnableIRQ(EXTI9_5_IRQn);                     // Interrupt enable
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->CR1 |= TIM_CR1_OPM;
	TIM6->ARR = 10000;
	TIM6->PSC = 16000;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);
	NVIC_SetPriority(TIM6_IRQn, 2);                // Priority set to 0
	NVIC_EnableIRQ(TIM6_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
	uint8_t buffer_send[1];
	
	if (EXTI->PR & EXTI_PR_PR6) {
		if (HALL_Data.HuntTime) {
			if (HALL_Data.SequenceCnt) {
				if (SYS_TICK_timeOut(1, HALL_Data.Time) < HALL_Timeout) {
					HALL_Data.SequenceCnt++;
					if (HALL_Data.SequenceCnt > HALL_SequenceNumber) {
						// Branie
						TIM6->CNT = 0;
						TIM6->CR1 |= TIM_CR1_CEN;
						HALL_Data.SequenceCnt = HALL_SequenceNumber;
						// Prevent from sending packets too often
						if (SYS_TICK_timeOut(1, HALL_Data.TimeOut) > 500) {
							HALL_Data.TimeOut = SYS_TICK_timeOut(0, 0);
							_BUZZER_alarm_start();
							HALL_Data.Result = TRUE;
							buffer_send[0] = MENU_RF_HallAlarm;
							RFM69W_sendWithRetry(0x00, buffer_send, 1, 15, 10);
//#ifdef USART_debug
//							USART_send("Fish caught!\n");
//#endif
						}
					} else {
//#ifdef USART_debug
//						USART_send("Roller moved with #");
//						USART_write_buf(HALL_Data.SequenceCnt, DEC);
//						USART_send(".\n");
//#endif						
					}
					HALL_Data.Time = SYS_TICK_timeOut(0, 0);
				} else {
					HALL_Data.SequenceCnt = 0;
//#ifdef USART_debug
//					USART_send("Roller moved with #0.\n");
//#endif
				}
			} else {
				HALL_Data.SequenceCnt++;
				HALL_Data.Time = SYS_TICK_timeOut(0, 0);

//#ifdef USART_debug
//				USART_send("Roller moved with #");
//				USART_write_buf(HALL_Data.SequenceCnt, DEC);
//				USART_send(".\n");
//#endif
			}
		}
		
		EXTI->PR |= EXTI_PR_PR6;
	}
}

void TIM6_IRQHandler()
{
	uint8_t buffer_send[1];
	
	if (TIM6->SR & TIM_SR_UIF) {
		_BUZZER_alarm_stop();
		HALL_Data.Result = FALSE;
		buffer_send[0] = MENU_RF_HallAlarmStop;
		RFM69W_sendWithRetry(0x00, buffer_send, 1, 15, 10);
//#ifdef USART_debug
//			USART_send("Hunt down.\n");
//#endif
		
		TIM6->SR &= ~(TIM_SR_UIF);
	}
}