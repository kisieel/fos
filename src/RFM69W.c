#include "RFM69W.h"
#include "USART.h"
#include "stm32l1xx.h"

// Available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define GATEWAYID              1
#define NODEID                 133
#define NETWORKID              100
#define FREQUENCY              RF69_868MHZ
#define ENCRYPTKEY             "FishingMonsters!" // Has to be same 16 characters/bytes on all nodes, not more not less!

#define RFM69W_CS_UP    GPIOA->ODR |= GPIO_ODR_ODR_15;
#define RFM69W_CS_DOWN  GPIOA->ODR &= ~GPIO_ODR_ODR_15;

// Private functions
void RFM69W_SPI_init(void);
void RFM69W_GPIO_init(void);
void RFM69W_REG_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);

void RFM69W_encrypt(const char* key);
void RFM69W_setHighPower(uint8_t isRFM69HW);

void RFM69W_FIFO_fill(unsigned char INDEX, unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA);
unsigned char RFM69W_SPI_write(unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA);
// End of private functions

// Private variables
typedef struct {
	uint16_t data;
	uint8_t type;
} RFM69W_Queue;

volatile struct {
	RFM69W_Queue buffer[RFM69W_FIFO_size];
	uint8_t head;
	uint8_t tail;
} RFM69W_FIFO;

volatile uint8_t RFM69W_mode;
uint8_t RFM69W_address;
uint8_t RFM69W_slaveSelectPin;
uint8_t RFM69W_interruptPin;
uint8_t RFM69W_interruptNum;

volatile uint8_t DATA[61];                // recv/xmit buf, including header & crc bytes
volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;                // should match _address
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED;            // should be polled immediately after sending a packet with ACK request
volatile int16_t RSSI;                    // most accurate RSSI during reception (closest to the reception)

volatile uint8_t _RFM69W_powerLevel = 31;
// End of Private variables

uint16_t _RFM69W_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{
	if (TYPE == RFM69W_read) {
		while(!(SPI1->SR & SPI_SR_TXE));
		while(SPI1->SR & SPI_SR_BSY);
		
		SPI1->CR1 |= SPI_CR1_SPE;
		RFM69W_CS_DOWN;
		
		SPI1->DR = (ADDRESS << 8) | 0x00;
		while(!(SPI1->SR & SPI_SR_TXE));
		while(!(SPI1->SR & SPI_SR_RXNE));
		
		DATA = SPI1->DR;
		while(SPI1->SR & SPI_SR_BSY);
		
		RFM69W_CS_UP;
		SPI1->CR1 &= ~SPI_CR1_SPE;
		return DATA;
	}
	return 0;
}

void _RFM69W_send(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{	
	if (TYPE == RFM69W_write)
		while (!(RFM69W_SPI_write(RFM69W_write, ADDRESS, DATA)));
	else
		while (!(RFM69W_SPI_write(RFM69W_read, ADDRESS, DATA)));
	
	// Check if FIFO is empty
	if ((RFM69W_FIFO.tail == (RFM69W_FIFO.head + 1)) || ((RFM69W_FIFO.tail == 0) && (RFM69W_FIFO.head == RFM69W_FIFO_size))) {
		SPI1->CR2 |= SPI_CR2_RXNEIE;
		SPI1->CR1 |= SPI_CR1_SPE;
		RFM69W_CS_DOWN;
		SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
	}
}

uint8_t RFM69W_SPI_write(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA) 
{
//	__disable_irq();
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
//	__enable_irq();
	
	return 1;
}

void RFM69W_FIFO_fill(uint8_t INDEX, uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{	
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
		buffer = SPI1->DR;
		RFM69W_CS_UP;
		
		if (RFM69W_FIFO.buffer[RFM69W_FIFO.head].type == RFM69W_read) {
			// Incomming handle
//			switch (RFM69W_FIFO.buffer[RFM69W_FIFO.head].data) {
//				case 1:
//					break;
//				case 2:
//					break;
//				case 3:
//					break;
//				case 4:
//					break;
//				case 5:
//					break;
//				case 6:
//					break;
//				case 7:
//					break;
//				case 8:
//					break;
//				case 9:
//					break;
//			}
		}
		
		if ((RFM69W_FIFO.head + 1) == RFM69W_FIFO_size)
			RFM69W_FIFO.head = 0;
		else
			RFM69W_FIFO.head++;
		
		if (RFM69W_FIFO.head == RFM69W_FIFO.tail) {
			while (SPI1->SR & SPI_SR_BSY);
			RFM69W_CS_UP;
			SPI1->CR1 &= ~SPI_CR1_SPE;
			SPI1->CR2 &= ~SPI_CR2_RXNEIE;
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
//	SPI1->CR2 |= SPI_CR2_RXNEIE;                    // Receive interrupt enable
	
	NVIC_EnableIRQ(SPI1_IRQn);
}

void RFM69W_REG_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{	
	do
		_RFM69W_send(RFM69W_write, REG_SYNCVALUE1, 0xAA);
	while
		(_RFM69W_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0xAA);
	
	do
		_RFM69W_send(RFM69W_write, REG_SYNCVALUE1, 0x55);
	while
		(_RFM69W_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0x55);

	_RFM69W_send(RFM69W_write, REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
	_RFM69W_send(RFM69W_write, REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
	_RFM69W_send(RFM69W_write, REG_BITRATEMSB, RF_BITRATEMSB_55555);
	_RFM69W_send(RFM69W_write, REG_BITRATELSB, RF_BITRATELSB_55555);
	_RFM69W_send(RFM69W_write, REG_FDEVMSB, RF_FDEVMSB_50000);
	_RFM69W_send(RFM69W_write, REG_FDEVLSB, RF_FDEVLSB_50000);
	_RFM69W_send(RFM69W_write, REG_FRFMSB, RF69_868MHZ);
	_RFM69W_send(RFM69W_write, REG_FRFMID, RF69_868MHZ);
	_RFM69W_send(RFM69W_write, REG_FRFLSB, RF69_868MHZ);
//	_RFM69W_send(RFM69W_write, REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
//	_RFM69W_send(RFM69W_write, REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
	_RFM69W_send(RFM69W_write, REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
	_RFM69W_send(RFM69W_write, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
	_RFM69W_send(RFM69W_write, REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF);
	_RFM69W_send(RFM69W_write, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
	_RFM69W_send(RFM69W_write, REG_RSSITHRESH, 220);
//	_RFM69W_send(RFM69W_write, REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE);
	_RFM69W_send(RFM69W_write, REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
	_RFM69W_send(RFM69W_write, REG_SYNCVALUE1, 0x2D);
	_RFM69W_send(RFM69W_write, REG_SYNCVALUE2, NETWORKID);
	_RFM69W_send(RFM69W_write, REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
	_RFM69W_send(RFM69W_write, REG_PAYLOADLENGTH, 66);
//	_RFM69W_send(RFM69W_write, REG_NODEADRS, NODEID);
	_RFM69W_send(RFM69W_write, REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
	_RFM69W_send(RFM69W_write, REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
	_RFM69W_send(RFM69W_write, REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);	
	
	RFM69W_encrypt(0);
	
	RFM69W_setHighPower(0);
	_RFM69W_setMode(RF69_MODE_STANDBY);
	
	while ((_RFM69W_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  
	// RFM69_WAKE_uC as a DIO0 pin in RFM69W module
	// PA10 input floating in module
	GPIO_config(0x0A, 10, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;    // A block for EXTI10
	EXTI->IMR |= EXTI_IMR_MR10;                       // Input line 10 selection (unmasking)
	EXTI->RTSR |= EXTI_RTSR_TR10;                     // Rising edge selection
	NVIC_SetPriority(EXTI15_10_IRQn, 0);              // Priority set to 0
	NVIC_EnableIRQ(EXTI15_10_IRQn);                   // Interrupt enable
	
  RFM69W_address = NODEID;
}

void RFM69W_encrypt(const char* key)
{
	uint8_t i;
	
	_RFM69W_setMode(RF69_MODE_STANDBY);
	
  if (key != 0) {
		for (i = 0; i < 16; i++)
			_RFM69W_send(RFM69W_write, REG_AESKEY1 + i, key[i]);
  }
	
	_RFM69W_send(RFM69W_write, REG_PACKETCONFIG2, (_RFM69W_send_poll(RFM69W_read, REG_PACKETCONFIG2, 0x00) & 0xFE) | (key ? 1 : 0));
}

void _RFM69W_setMode(uint8_t newMode)
{
  if (newMode == RFM69W_mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
			_RFM69W_send(RFM69W_write, REG_OPMODE, (_RFM69W_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
			_RFM69W_send(RFM69W_write, REG_OPMODE, (_RFM69W_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
			_RFM69W_send(RFM69W_write, REG_OPMODE, (_RFM69W_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
			_RFM69W_send(RFM69W_write, REG_OPMODE, (_RFM69W_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
			_RFM69W_send(RFM69W_write, REG_OPMODE, (_RFM69W_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_SLEEP);
      break;
  }
	
	while (RFM69W_mode == RF69_MODE_SLEEP && (_RFM69W_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  RFM69W_mode = newMode;
}

void RFM69W_setHighPower(uint8_t _isRFM69HW) {
  _RFM69W_send(RFM69W_write, REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    _RFM69W_send(RFM69W_write, REG_PALEVEL, (_RFM69W_send_poll(RFM69W_read, REG_PALEVEL, 0x00) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    _RFM69W_send(RFM69W_write, REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _RFM69W_powerLevel); // enable P0 only
}

void _RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
	RFM69W_REG_init(FREQUENCY, NODEID, NETWORKID);
}
