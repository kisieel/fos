#include "RFM69W.h"
#include "stm32l1xx.h"

volatile RFM69W_Queue RFM69W_FIFO[10];

uint8_t RFM69W_SPI_write(uint8_t TYPE, uint8_t ADRESS, uint8_t DATA) {
	if (RFM69W_FIFO[0].Flag_address == 0 & RFM69W_FIFO[0].Flag_data == 0) {
		RFM69W_FIFO_Fill(0, TYPE, ADRESS, DATA);
		SPI1->DR = RFM69W_FIFO[0].Address;
		return 1;
	} else {
		if (RFM69W_FIFO[1].Flag_address == 0 & RFM69W_FIFO[1].Flag_data == 0) {
			RFM69W_FIFO_Fill(1, TYPE, ADRESS, DATA);
			return 1;
		} else {
			return 0;
		}
	}
}

void RFM69W_FIFO_Fill(uint8_t INDEX, uint8_t TYPE, uint8_t ADRESS, uint8_t DATA)
{
	if (TYPE) {
		ADRESS |= 0x80;
		RFM69W_FIFO[INDEX].Type = 1;
	} else {
		RFM69W_FIFO[INDEX].Type = 0; 
	}
	RFM69W_FIFO[INDEX].Flag_address;
	RFM69W_FIFO[INDEX].Flag_data;
	RFM69W_FIFO[INDEX].Address = ADRESS;
	RFM69W_FIFO[INDEX].Data = DATA;
}

void RFM69W_FIFO_Rollup()
{
	if (RFM69W_FIFO[1].Flag_address == 1 & RFM69W_FIFO[1].Flag_data == 1) {
		RFM69W_FIFO[0] = RFM69W_FIFO[1];
		RFM69W_FIFO[1].Flag_address = 0;
		RFM69W_FIFO[1].Flag_data = 0;
	}
	
	if (RFM69W_FIFO[2].Flag_address == 1 & RFM69W_FIFO[2].Flag_data == 1) {
		RFM69W_FIFO[1] = RFM69W_FIFO[2];
		RFM69W_FIFO[2].Flag_address = 0;
		RFM69W_FIFO[2].Flag_data = 0;
	}
}

void SPI1_IRQHandler()
{
	if (SPI1->SR & SPI_SR_RXNE) {
		if (RFM69W_FIFO[0].Type == 0 & RFM69W_FIFO[0].Flag_address == 0) {
			// cos przyszlo co trzeba obsluzyc, wyslij raport
		}
		SPI1->SR &= ~(SPI_SR_RXNE);
	}
	if (SPI1->SR & SPI_SR_TXE) {
		if (RFM69W_FIFO[0].Flag_address == 1) {
			SPI1->DR = RFM69W_FIFO[0].Flag_data;
			RFM69W_FIFO[0].Flag_address = 0;
		} else {
			if (RFM69W_FIFO[0].Flag_data == 1) {
				RFM69W_FIFO[0].Flag_data = 0;
				RFM69W_FIFO_Rollup();
				if (RFM69W_FIFO[0].Flag_address == 1 & RFM69W_FIFO[0].Flag_data == 1) {
					SPI1->DR = RFM69W_FIFO[0].Address;
				}
			}
		}
		SPI1->SR &= ~(SPI_SR_TXE);
	}
}

void RFM69W_GPIO_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                // Clock for GPIOA 
	
	// NSS/MISO/MOSI/SCK = PA4/PA6/PA7/PA5 pin configuration as 
	// (MISO) input floating, 
	// (MOSI) alternate function push-pull, 
	// (SCK)  alternate function push-pull
	// (NSS)  alternate function push-pull
	
	GPIOA->MODER |= GPIO_MODER_MODER4_1               //  Alternate function mode
	             | GPIO_MODER_MODER5_1
	             | GPIO_MODER_MODER7_1;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER6);             // Input floating
	
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4               // Output push-pull
	              | GPIO_OTYPER_OT_5
	              | GPIO_OTYPER_OT_7);
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0        // 2 MHz Low Speed
	               | GPIO_OSPEEDER_OSPEEDR5_0
	               | GPIO_OSPEEDER_OSPEEDR6_0
								 | GPIO_OSPEEDER_OSPEEDR7_0;
	
}

void RFM69W_SPI_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;             // Clock for SPI
	
	SPI1->CR1 |= SPI_CR1_BR_1                       // Prescaler: 8, 72MHz/8= 9 MHz
	          | SPI_CR1_BR_0
	          | SPI_CR1_MSTR                        // Master mode
						| SPI_CR1_SSI;
	
	SPI1->CR1 &= ~(SPI_CR1_CPOL                     // CPOL = 0
	          | SPI_CR1_CPHA                        // CPHA = 0
	          | SPI_CR1_SSM                         // Hardware NSS managment
	          | SPI_CR1_DFF                         // 8-bit
	          | SPI_CR1_LSBFIRST);                  // LSB first
	
	SPI1->CR2 |= SPI_CR2_TXEIE                      // Transmit interrupt enable
	          | SPI_CR2_RXNEIE                      // Receive interrupt enable
	          | SPI_CR2_SSOE;                       // 
	
	NVIC_EnableIRQ(SPI1_IRQn);
	
	SPI1->CR1 |= SPI_CR1_SPE;                       // SPI enable
}

void RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
}