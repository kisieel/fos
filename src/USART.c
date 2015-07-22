#include "stm32l1xx.h"
#include "USART.h"

volatile struct {
	USART_Queue buffer[USART_FIFO_size];
	uint8_t head;
	uint8_t tail;
} USART_FIFO;

uint8_t UART_write(char DATA) 
{
	if (USART_FIFO.tail == (USART_FIFO_size - 1)) {
		if (USART_FIFO.head == 0) 
			return 0;
	} else {
		if (USART_FIFO.head == ++USART_FIFO.tail)
			return 0;
	}
	
	USART_FIFO.buffer[USART_FIFO.tail].data = DATA;

	if (USART_FIFO.tail == USART_FIFO.head)
		USART1->DR = USART_FIFO.buffer[USART_FIFO.head].data;
	
	if (USART_FIFO.tail++ == USART_FIFO_size) {
		USART_FIFO.tail = 0;
	} else {
		USART_FIFO.tail++;
	}
	return 1;
}

void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_TXE) {
		if (USART_FIFO.head++ == USART_FIFO_size)
			USART_FIFO.head = 0;
		else
			USART_FIFO.head++;
		
		if (USART_FIFO.head == USART_FIFO.tail)
			USART1->DR = USART_FIFO.buffer[USART_FIFO.head].data;
		
		USART1->SR &= ~(USART_SR_TXE);
	}
	
	if (USART1->SR & USART_SR_RXNE) {
		
		
		USART1->SR &= ~(USART_SR_RXNE);
	}
}

void USART_putchar(char ch)
{
	while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = ch;
}

void USART_send(char * text)
{
	while (*text!='\0')
	{
//		USART_write(*text);
		USART_putchar(*text);
		text++;
	}
}

void USART_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                  // Clock for GPIOB 
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;               // Clock for USART1
	
//	GPIO_config(0x0B, 6, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
//	GPIO_config(0x0B, 7, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
	
	GPIOB->MODER |= GPIO_MODER_MODER6_1                 // Alternate function mode
	              | GPIO_MODER_MODER7_1;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_ODR_6                // Push-pull
	                 | GPIO_OTYPER_ODR_7);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6          // Very low speed 400 kHz
	                  | GPIO_OSPEEDER_OSPEEDR7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6                 // No pull-up/pull down
	                | GPIO_PUPDR_PUPDR7);
	GPIOB->AFR[0] |= (GPIO_AFRL_AFRL6 & (7 << 4*6))     // AF7- USART1 TX/RX
	               | (GPIO_AFRL_AFRL7 & (7 << 4*7));
	
	USART1->BRR = (SystemCoreClock / 19200);            // Baudrate = (sysclock/baudrate)
	
	USART1->CR1 |= USART_CR1_UE                         // USART enable
	             | USART_CR1_TE                         // Transmit enable
	             | USART_CR1_RE                         // Receive enable
							 | USART_CR1_RXNEIE
							 | USART_CR1_TXEIE;
	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 1);
}
