#include "RFM69W.h"
#include "USART.h"
#include "stm32l1xx.h"

#define RFM69W_CS_UP    GPIOA->ODR |= GPIO_ODR_ODR_15;
#define RFM69W_CS_DOWN  GPIOA->ODR &= ~GPIO_ODR_ODR_15;

// Private functions
void RFM69W_SPI_init(void);
void RFM69W_GPIO_init(void);
void RFM69W_REG_init(void);

void RFM69W_FIFO_fill(unsigned char INDEX, unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA);
unsigned char RFM69W_SPI_write(unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA);
// End of private functions

typedef struct {
	uint16_t data;
	uint8_t type;
	uint8_t flag;
} RFM69W_Queue;

volatile struct {
	RFM69W_Queue buffer[RFM69W_FIFO_size];
	uint8_t head;
	uint8_t tail;
} RFM69W_FIFO;

void _RFM69W_send(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{	
	if (TYPE == RFM69W_write)
		while (!(RFM69W_SPI_write(RFM69W_write, ADDRESS, DATA)));
	else
		while (!(RFM69W_SPI_write(RFM69W_read, ADDRESS, DATA)));
	
	// Check if FIFO is empty
	if ((RFM69W_FIFO.tail == (RFM69W_FIFO.head + 1)) || ((RFM69W_FIFO.tail == 0) && (RFM69W_FIFO.head == RFM69W_FIFO_size))) {
		
		SPI1->CR1 |= SPI_CR1_SPE;
		RFM69W_CS_DOWN;
		SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
	}
}

uint8_t RFM69W_SPI_write(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA) 
{
	__disable_irq();
	// Check if FIFO is full
	if (RFM69W_FIFO.tail == (RFM69W_FIFO_size - 1)) {
		if (RFM69W_FIFO.head == 0) 
			return 0;
	} else {
		if (RFM69W_FIFO.head == (RFM69W_FIFO.tail + 1))
			return 0;
	}
	
	// If FIFO is not full, fill the TAIL buffer
	RFM69W_FIFO_fill(RFM69W_FIFO.tail, TYPE, ADDRESS, DATA);
	
	// Increment TAIL
	if ((RFM69W_FIFO.tail + 1) == RFM69W_FIFO_size) {
		RFM69W_FIFO.tail = 0;
	} else {
		RFM69W_FIFO.tail++;
	}
	__enable_irq();
	
	return 1;
}

void RFM69W_FIFO_fill(uint8_t INDEX, uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{
	// Set the flag 
	RFM69W_FIFO.buffer[INDEX].flag = 1;
	
	switch (TYPE) {
		case RFM69W_write:
			RFM69W_FIFO.buffer[INDEX].type = RFM69W_write;
			RFM69W_FIFO.buffer[INDEX].data = ((ADDRESS | 0x80) << 8 ) | DATA;
			break;
		case RFM69W_read:
			RFM69W_FIFO.buffer[INDEX].type = RFM69W_read;
			RFM69W_FIFO.buffer[INDEX].data = (ADDRESS << 8 ) | DATA;
			break;
	}
	
}

void SPI1_IRQHandler()
{
	uint16_t buffer;
	
	if (SPI1->SR & SPI_SR_RXNE) {
		RFM69W_CS_UP;
		buffer = SPI1->DR;
		
		if (RFM69W_FIFO.buffer[RFM69W_FIFO.head].type == RFM69W_address_read) {
			// Incomming handle
		}
		
		if ((RFM69W_FIFO.head + 1) == RFM69W_FIFO_size)
			RFM69W_FIFO.head = 0;
		else
			RFM69W_FIFO.head++;
		
		if (RFM69W_FIFO.head == RFM69W_FIFO.tail) {
			while (SPI1->SR & SPI_SR_BSY);
			RFM69W_CS_UP;
			SPI1->CR1 &= ~SPI_CR1_SPE;
		} else {
			RFM69W_CS_DOWN;
			SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
		}
	}
}


void EXTI15_10_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR10) {
		
		EXTI->PR |= EXTI_PR_PR10;
	}
}

void RFM69W_GPIO_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN                 // Clock for GPIOB 
               | RCC_AHBENR_GPIOAEN;                // Clock for GPIOA
	
	// RFM69_WAKE_uC as a DIO0 pin in RFM69W module
	// PA10 input floating in module
	GPIO_config(0x0A, 10, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;    // A block for EXTI10
	EXTI->IMR |= EXTI_IMR_MR10;                       // Input line 10 selection (unmasking)
	EXTI->RTSR |= EXTI_RTSR_TR10;                     // Rising edge selection
	NVIC_SetPriority(EXTI15_10_IRQn, 0);              // Priority set to 0
	NVIC_EnableIRQ(EXTI15_10_IRQn);                   // Interrupt enable
	
	// RFM69_Reset_uC as a RST pin in RFM69W module
	// PA9 in module
	GPIO_config(0x0A, 9, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	
	// SPI SCK/MISO/MOSI/CS = PB3/PB4/PB5/PA15 pin configuration as (according to RM0008 document)
	// (SCK)  alternate function push-pull
	// (MISO) input pull-up, 
	// (MOSI) alternate function push-pull, 
	// (CS)   general purpose push-pull.
	
	// SCK and MOSI configuration:
	GPIO_config(0x0B, 3, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_2M, GPIO_AF_AF5);
	GPIO_config(0x0B, 5, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_2M, GPIO_AF_AF5);
	
	// MISO confuguration:
	GPIO_config(0x0B, 4, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_2M, GPIO_AF_AF5);
	
	// CS configuration:
	GPIO_config(0x0A, 15, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	
	RFM69W_CS_UP;                                     // CS up
}

void RFM69W_SPI_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;             // Clock for SPI
	
	SPI1->CR1 |= SPI_CR1_BR_2                       // Prescaler: 32, 32 MHz/32 = 1 MHz
	           | SPI_CR1_MSTR                       // Master mode
	           | SPI_CR1_DFF
						 | SPI_CR1_SSM                        // Software slave managment
	           | SPI_CR1_SSI;
	
//	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->CR2 |= SPI_CR2_RXNEIE;                    // Receive interrupt enable
	
	NVIC_EnableIRQ(SPI1_IRQn);
}

void RFM69W_REG_init(void)
{
	
}

void _RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
	RFM69W_REG_init();
}
