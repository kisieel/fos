#include "stm32l1xx.h"
#include "USART.h"
#include "FLOAT.h"

// Private functions
unsigned char USART_write(char);
void USART_putchar(char);
// End of private functions

typedef struct {
	uint8_t data;
} USART_Queue;

volatile struct {
	USART_Queue buffer[USART_FIFO_size];
	uint8_t head;
	uint8_t tail;
} USART_FIFO;

void _USART_write_buf(uint32_t DATA, uint8_t TYPE)
{
	uint8_t buf[20];
	
	if (TYPE == DEC) {
		_dbl2stri(buf, DATA, 0);
		_USART_send(buf);
		return;
	}
	
	if (TYPE == BIN) {
		_dbl2stri(buf, _decimal_binary(DATA), 0);
		_USART_send(buf);
	}
}

unsigned char USART_write(char DATA) 
{
	if (USART_FIFO.tail == (USART_FIFO_size - 1)) {
		if (USART_FIFO.head == 0) 
			return 0;
	} else {
		if (USART_FIFO.head == (USART_FIFO.tail + 1))
			return 0;
	}
	
	USART_FIFO.buffer[USART_FIFO.tail].data = DATA;

	if (USART_FIFO.tail == USART_FIFO.head)
		USART1->DR = USART_FIFO.buffer[USART_FIFO.head].data;
	
	if ((USART_FIFO.tail + 1) == USART_FIFO_size) {
		USART_FIFO.tail = 0;
	} else {
		USART_FIFO.tail++;
	}
	return 1;
}

void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_TC) {
		if ((USART_FIFO.head + 1) == USART_FIFO_size)
			USART_FIFO.head = 0;
		else
			USART_FIFO.head++;
		
		if (!(USART_FIFO.head == USART_FIFO.tail))
			USART1->DR = USART_FIFO.buffer[USART_FIFO.head].data;
		
		USART1->SR &= ~(USART_SR_TC);
	}
	
	if (USART1->SR & USART_SR_RXNE) {
		
		
		USART1->SR &= ~(USART_SR_RXNE);
	}
}

void _USART_send(char * text)
{
	while (*text != '\0')
	{
		while(!(USART_write(*text)));
		text++;
	}
}

void _USART_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                  // Clock for GPIOB 
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;               // Clock for USART1
	
	GPIO_config(0x0B, 6, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
	GPIO_config(0x0B, 7, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
	
	USART1->BRR = (SystemCoreClock / 19200);            // Baudrate = (sysclock/baudrate)
	
	USART1->CR1 |= USART_CR1_UE                         // USART enable
	             | USART_CR1_TE                         // Transmit enable
	             | USART_CR1_RE                         // Receive enable
							 | USART_CR1_RXNEIE
							 | USART_CR1_TCIE;
	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 1);
	
	USART_FIFO.head = 0;
}
