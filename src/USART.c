#include "stm32l1xx.h"
#include "USART.h"
#include "FLOAT.h"

//#define USART_INTERRUPT  1
#define USART_POOLING    1

#define USART_FIFO_size  1000

#define USART_FREE   0
#define USART_NOFREE 1

// Private functions
void USART_init(void);
void USART_send(char *);
void USART_write_buf(uint32_t DATA, uint8_t TYPE);

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

void USART_write_buf(uint32_t DATA, uint8_t TYPE)
{
	uint8_t buf[25];
	
	if (TYPE == DEC) {
		_dbl2stri(buf, DATA, 0);
		USART_send(buf);
		return;
	}
	
	if (TYPE == BIN) {
		_dbl2stri(buf, _decimal_binary(DATA), 0);
		USART_send(buf);
	}
}

void USART_putchar(char ch)
{
	while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = ch;
}

uint8_t USART_write(char DATA) 
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
	
//	if (USART1->SR & USART_SR_RXNE) {
//		
//		
//		USART1->SR &= ~(USART_SR_RXNE);
//	}
}

void USART_send(char * text)
{
	while (*text != '\0')
	{
#ifdef USART_INTERRUPT
//		while(!(USART_write(*text)));
#endif
#ifdef USART_POOLING
		USART_putchar(*text);
#endif 
		text++;
	}
}

void USART_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                  // Clock for GPIOB 
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;               // Clock for USART1
	
	GPIO_config(0x0B, 6, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
	GPIO_config(0x0B, 7, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, GPIO_AF_AF7);
	
	USART1->BRR = (SystemCoreClock / 115200);            // Baudrate = (sysclock/baudrate)
	
	USART1->CR1 |= USART_CR1_UE                         // USART enable
	             | USART_CR1_TE                         // Transmit enable
	             | USART_CR1_RE                         // Receive enable
#ifdef USART_POOLING
		;
#endif
#ifdef USART_INTERRUPT
							 | USART_CR1_RXNEIE
							 | USART_CR1_TCIE;

	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 0);
	
	USART_FIFO.head = 0;
#endif
}
