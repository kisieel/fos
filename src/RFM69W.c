#include "RFM69W.h"
#include "stm32l1xx.h"

volatile struct {
	RFM69W_Queue buffer[RFM69W_FIFO_size];
	uint8_t actual;
	uint8_t head;
	uint8_t tail;
} RFM69W_FIFO;

void RFM69W_SPI_send(unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA)
{
	if (TYPE == RFM69W_write)
		while (!(RFM69W_SPI_write(RFM69W_write, ADDRESS)));
	else
		while (!(RFM69W_SPI_write(RFM69W_write_ad, ADDRESS)));
	while (!(RFM69W_SPI_write(TYPE, DATA)));
}

uint8_t RFM69W_SPI_write(unsigned char TYPE, unsigned char DATA) 
{
	if (RFM69W_FIFO.tail == (RFM69W_FIFO_size - 1)) {
		if (RFM69W_FIFO.head == 0) 
			return 0;
	} else {
		if (RFM69W_FIFO.head == (RFM69W_FIFO.tail + 1))
			return 0;
	}
	
	RFM69W_FIFO_fill(RFM69W_FIFO.tail, TYPE, DATA);

	if (RFM69W_FIFO.tail == RFM69W_FIFO.head)
		SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
	
	if ((RFM69W_FIFO.tail + 1) == RFM69W_FIFO_size) {
		RFM69W_FIFO.tail = 0;
		RFM69W_FIFO.buffer[0].type_prev = RFM69W_FIFO.buffer[RFM69W_FIFO_size - 1].type;
	} else {
		RFM69W_FIFO.tail++;
		RFM69W_FIFO.buffer[RFM69W_FIFO.tail].type_prev = RFM69W_FIFO.buffer[RFM69W_FIFO.tail - 1].type;
	}
	
	return 1;
}

void RFM69W_FIFO_fill(unsigned char INDEX, unsigned char TYPE, unsigned char DATA)
{
	if (TYPE == RFM69W_write | TYPE == RFM69W_write_ad) {
		RFM69W_FIFO.buffer[INDEX].type = RFM69W_write;
		if (TYPE == RFM69W_write)
			DATA |= 0x80;
	} else {
		RFM69W_FIFO.buffer[INDEX].type = RFM69W_read; 
	}
	RFM69W_FIFO.buffer[INDEX].data = DATA;
}

void SPI1_IRQHandler()
{
	if (SPI1->SR & SPI_SR_RXNE) {
		if (RFM69W_FIFO.buffer[RFM69W_FIFO.head].type_prev == RFM69W_read) {
			// cos przyszlo co trzeba obsluzyc, wyslij raport
		}
		SPI1->SR &= ~(SPI_SR_RXNE);
	}
	if (SPI1->SR & SPI_SR_TXE) {
		if ((RFM69W_FIFO.head + 1) == RFM69W_FIFO_size)
			RFM69W_FIFO.head = 0;
		else
			RFM69W_FIFO.head++;
		
		if (RFM69W_FIFO.head == RFM69W_FIFO.tail)
			SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
		
		SPI1->SR &= ~(SPI_SR_TXE);
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
	GPIO_config(0x0B, 4, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	// CS configuration:
	GPIO_config(0x0A, 15, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	
	RFM69W_CS_UP;                                     // CS up
}

void RFM69W_SPI_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;             // Clock for SPI
	
	SPI1->CR1 |= SPI_CR1_BR_2                       // Prescaler: 32, 32 MHz/32 = 1 MHz
	           | SPI_CR1_MSTR                       // Master mode
						 | SPI_CR1_SSM;                       // Software slave managment
	
	SPI1->CR2 |= SPI_CR2_TXEIE                      // Transmit interrupt enable
	          | SPI_CR2_RXNEIE                      // Receive interrupt enable
	          | SPI_CR2_SSOE;                       // 
	
	NVIC_EnableIRQ(SPI1_IRQn);
	
	SPI1->CR1 |= SPI_CR1_SPE;                       // SPI enable
}

void RFM69W_REG_init(void)
{
	
}

void RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
	RFM69W_REG_init();
}