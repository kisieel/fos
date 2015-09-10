#include "main.h"

#define NODEID                  0x01
#define NETWORKID               100
#define ENCRYPTKEY              "FishingMonsters!" // Has to be same 16 characters/bytes on all nodes, not more not less!

// Available frequency bands
#define RF69_868MHZ             86
#define FREQUENCY               RF69_868MHZ
#define RF69_BROADCAST_ADDR     255

#define RF69_CSMA_LIMIT_MS      1000
#define RF69_TX_LIMIT_MS        1000
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access

#define RF69_FSTEP              61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

#define RF69_MODE_SLEEP         0  // XTAL OFF
#define RF69_MODE_STANDBY       1  // XTAL ON
#define RF69_MODE_SYNTH         2  // PLL ON
#define RF69_MODE_RX            3  // RX MODE
#define RF69_MODE_TX            4  // TX MODE

#define DIO0                    0
#define DIO1                    1
#define DIO2                    2
#define DIO3                    3
#define DIO4                    4
#define DIO5                    5

#define true                    1
#define false                   0

//             DIO_0               DIO_1     DIO_2        DIO_3       DIO_4     DIO_5 
// Mapping     00/01               11        00           10          01        11
// SLEEP       -/-                 -         FifoNotEmpty -           -         Mode_Ready               
// STANDBY     -/-                 -         FifoNotEmpty -           -         Mode_Ready               
// FS          -/-                 PllLock   FifoNotEmpty -           -         Mode_Ready               
// RX          CrcOk/PayloadReady  Timeout   FifoNotEmpty SyncAddress RSSI      Mode_Ready               
// TX          PacketSent/TxReady  PllLock   FifoNotEmpty -           TxReady   Mode_Ready  

// IO FLAG checks

// DIO0
#define PacketSent_Check       !(GPIOC->IDR & GPIO_IDR_IDR_13)
#define CrcOk_Check            PacketSent_Check
#define PayloadReady_Check     PacketSent_Check

// DIO1
#define PllLock_Check          !(GPIOB->IDR & GPIO_IDR_IDR_11)
#define Timeout_Check          PllLock_Check

// DIO2
#define FifoNotEmpty_Check     !(GPIOB->IDR & GPIO_IDR_IDR_12)

// DIO3
#define SyncAddress_Check      !(GPIOB->IDR & GPIO_IDR_IDR_14)

// DIO4
#define TxReady_Check          !(GPIOB->IDR & GPIO_IDR_IDR_15)
#define RSSI_Check             TxReady_Check

//DIO5
#define ModeReady_Check        !(GPIOA->IDR & GPIO_IDR_IDR_8)

// Minimum size: 3
#define RFM69W_FIFO_size        20

#define RFM69W_CS_UP            GPIOA->ODR |= GPIO_ODR_ODR_15; SPI1->CR1 &= ~SPI_CR1_SPE; 
#define RFM69W_CS_DOWN          SPI1->CR1 |= SPI_CR1_SPE; GPIOA->ODR &= ~GPIO_ODR_ODR_15;

//__enable_irq(); __disable_irq(); 

//#define RFM69W_write            1
//#define RFM69W_read             0

// Private functions

/**
  * @brief  RFM69W oriented functions       
  */
uint8_t  RFM69W_REG_init(void);

uint8_t  RFM69W_ACKReceived(uint8_t fromNodeID);
void     RFM69W_sendACK(const void* buffer, uint8_t bufferSize);

uint8_t  RFM69W_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);
void     RFM69W_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK);
uint8_t  RFM69W_canSend(void);
uint8_t  RFM69W_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK, uint8_t sendACK);

void     EXTI15_10_IRQHandler(void);
void     RFM69W_interruptHandler(void);
void     RFM69W_receiveBegin(void);

int16_t  RFM69W_readRSSI(uint8_t forceTrigger);
void     RFM69W_readAllRegs(void);
void     RFM69W_setMode(uint8_t newMode);
void     RFM69W_setMap(uint8_t DIO, uint8_t MAP);
void     RFM69W_encrypt(const char* key);
void     RFM69W_reset(void);

void     RFM69W_listenModeON(void);
void     RFM69W_listenModeOFF(void);
	
/**
  * @brief  SPI and STM32L1 oriented functions
  */

uint8_t  RFM69W_SPI_send_8poll(uint8_t DATA);
uint16_t RFM69W_SPI_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);
void     RFM69W_SPI_send(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);
uint8_t  RFM69W_SPI_write(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);

void RFM69W_SPI_set_bit(uint8_t ADDRESS, uint8_t BITS);
void RFM69W_SPI_clear_bit(uint8_t ADDRESS, uint8_t BITS);

void     RFM69W_GPIO_init(void);
void     RFM69W_SPI_init(void);
void     RFM69W_init(void);

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

volatile RFM69W RFM69W_Data;


// End of Private variables

/**
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  * @brief RFM69W oriented functions  *  *  *  *
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  */

/**
  * @note 
  */
uint8_t RFM69W_REG_init(void)
{
	uint8_t buf[10];
	uint32_t i;
	
	RFM69W_reset();
	
	do
		RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE1, 0xAA);
	while
		(RFM69W_SPI_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0xAA);
	
	do
		RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE1, 0x55);
	while
		(RFM69W_SPI_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0x55);
	
	RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00);
	//
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
	
	RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00);
	
	//
	RFM69W_SPI_send_poll(RFM69W_write, REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
	
	// default: 4.8 KBPS
	RFM69W_SPI_send_poll(RFM69W_write, REG_BITRATEMSB, RF_BITRATEMSB_55555);
	// default: 4.8 KBPS
	RFM69W_SPI_send_poll(RFM69W_write, REG_BITRATELSB, RF_BITRATELSB_55555);
	
	// default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
	RFM69W_SPI_send_poll(RFM69W_write, REG_FDEVMSB, RF_FDEVMSB_50000);
	// default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
	RFM69W_SPI_send_poll(RFM69W_write, REG_FDEVLSB, RF_FDEVLSB_50000);
	
	// 868 MHz
	RFM69W_SPI_send_poll(RFM69W_write, REG_FRFMSB, RF_FRFMSB_868);
	// 868 MHz
	RFM69W_SPI_send_poll(RFM69W_write, REG_FRFMID, RF_FRFMID_868);
	// 868 MHz
	RFM69W_SPI_send_poll(RFM69W_write, REG_FRFLSB, RF_FRFLSB_868);
	
	// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
  // +17dBm and +20dBm are possible on RFM69HW
  // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
  // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
  // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
//	RFM69W_SPI_send_poll(RFM69W_write, REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
	//  over current protection (default is 95mA)
//	RFM69W_SPI_send_poll(RFM69W_write, REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
	// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
	// for BR-19200 { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
	RFM69W_SPI_send_poll(RFM69W_write, REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);

//             DIO_0               DIO_1     DIO_2        DIO_3       DIO_4     DIO_5 
// Mapping     00/01               11        00           10          01        11
// SLEEP       -/-                 -         FifoNotEmpty -           -         Mode_Ready               
// STANDBY     -/-                 -         FifoNotEmpty -           -         Mode_Ready               
// FS          -/-                 PllLock   FifoNotEmpty -           -         Mode_Ready               
// RX          CrcOk/PayloadReady  Timeout   FifoNotEmpty SyncAddress RSSI      Mode_Ready               
// TX          PacketSent/TxReady  PllLock   FifoNotEmpty -           TxReady   Mode_Ready               

	// DIO0 is the only IRQ we're using
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 
	                                                  | RF_DIOMAPPING1_DIO1_01
																										| RF_DIOMAPPING1_DIO2_00
																										| RF_DIOMAPPING1_DIO3_10);
																										
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_01
	                                                  | RF_DIOMAPPING2_CLKOUT_OFF
																										| RF_DIOMAPPING2_DIO5_11);
	
	// DIO0 RFM69_WAKE_uC init
	GPIO_config(0x0C, 13, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;    // C block for EXTI13
	EXTI->IMR |= EXTI_IMR_MR13;                       // Input line 13 selection (unmasking)
	EXTI->RTSR |= EXTI_RTSR_TR13;                     // Rising edge selection
	NVIC_SetPriority(EXTI15_10_IRQn, 1);              // Priority set to 0
	NVIC_EnableIRQ(EXTI15_10_IRQn);                   // Interrupt enable
	
	// DIO1 ModeReady init
	GPIO_config(0x0B, 11, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	// DIO2 ModeReady init
	GPIO_config(0x0B, 12, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	// DIO3 ModeReady init
	GPIO_config(0x0B, 14, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	// DIO4 ModeReady init
	GPIO_config(0x0B, 15, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);
	
	// DIO5 ModeReady init
	GPIO_config(0x0A, 8, GPIO_MODE_Input, GPIO_PULL_Pullup, 0, 0, 0);	
	
	// writing to this bit ensures that the FIFO & status flags are reset
	RFM69W_SPI_send_poll(RFM69W_write, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
	
	// must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
	RFM69W_SPI_send_poll(RFM69W_write, REG_RSSITHRESH, 220);
	
	//  default 3 preamble bytes 0xAAAAAA
//	RFM69W_SPI_send_poll(RFM69W_write, REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE);

	//
	RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
	
	// attempt to make this compatible with sync1 byte of RFM12B lib
	RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE1, 0x2D);
	
	// NETWORK ID
	RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE2, NETWORKID);
	
	//
	RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
	
	// in variable length mode: the max frame size, not used in TX
	RFM69W_SPI_send_poll(RFM69W_write, REG_PAYLOADLENGTH, 66);
	
	// turned off because we're not using address filtering
//	RFM69W_SPI_send_poll(RFM69W_write, REG_NODEADRS, NODEID);
	
	// TX on FIFO not empty
	RFM69W_SPI_send_poll(RFM69W_write, REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
	
	// RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
	// RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
//	RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
	
	// run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
	RFM69W_SPI_send_poll(RFM69W_write, REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);	
	
	// Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
	RFM69W_encrypt(ENCRYPTKEY);
	
	// Over current protection off
	RFM69W_SPI_send_poll(RFM69W_write, REG_OCP, RF_OCP_OFF);
	
	// Enable P0 only
  RFM69W_SPI_send_poll(RFM69W_write, REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | 31);
	
	RFM69W_setMode(RF69_MODE_STANDBY);
  
	// RSSI and SyncAddressMatch criteria, 64 us RX resolution, 4.1 ms Idle resolution
	// End: Module stays in Rx mode. Listen mode stops and must be disabled (RFM69W_listenModeOFF must be called)
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN1, RF_LISTEN1_CRITERIA_RSSIANDSYNC | RF_LISTEN1_RESOL_RX_64 | RF_LISTEN1_RESOL_IDLE_4100 | RF_LISTEN1_END_00);
	// Idle time ~100 ms = 4.1 ms * 24
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN2, 24);
	// RX time ~10 ms = 64 us * 156
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN3, 156);
	
  RFM69W_Data.RFM69W_Address = NODEID;
	
	RFM69W_listenModeON();
	
//	buf[0] = 'L';
//	buf[1] = 'O';
//	buf[2] = 'L';
//	
//	RFM69W_sendWithRetry(0x12, buf, 3, 10, 20);

	return true;
}

/**
  * @note Function reads mode register, clears it's 3 bits and sets new 3 ones
  */
void RFM69W_setMode(uint8_t newMode)
{
  switch (newMode) {
    case RF69_MODE_TX:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE ,(RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & ~RF_OPMODE_MODEMASK) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE ,(RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & ~RF_OPMODE_MODEMASK) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE ,(RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & ~RF_OPMODE_MODEMASK) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE ,(RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & ~RF_OPMODE_MODEMASK) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE ,(RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & ~RF_OPMODE_MODEMASK) | RF_OPMODE_SLEEP);
      break;
  }
	
	while (newMode == RF69_MODE_SLEEP && ModeReady_Check);
}

/**
  * @note Function reads map register, clears it's 2 bits and sets new 2 ones
  */
void RFM69W_setMap(uint8_t DIO, uint8_t MAP)
{
	switch (DIO) {
		case DIO0:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING1, 0x00) & ~RF_DIOMAPPING1_DIO0_11) | MAP);
			break;
		case DIO1:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING1, 0x00) & ~RF_DIOMAPPING1_DIO1_11) | MAP);
			break;
		case DIO2:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING1, 0x00) & ~RF_DIOMAPPING1_DIO2_11) | MAP);
			break;
		case DIO3:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING1, 0x00) & ~RF_DIOMAPPING1_DIO3_11) | MAP);
			break;
		case DIO4:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING2 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING2, 0x00) & ~RF_DIOMAPPING2_DIO4_11) | MAP);			
			break;
		case DIO5:
			RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING2 ,(RFM69W_SPI_send_poll(RFM69W_read, REG_DIOMAPPING2, 0x00) & ~RF_DIOMAPPING2_DIO5_11) | MAP);
			break;
	}
}

/**
  * @note Should be polled immediately after sending a packet with ACK request.
  */
uint8_t RFM69W_ACKReceived(uint8_t fromNodeID) 
{
  if ((RFM69W_Data.SenderID == fromNodeID) && RFM69W_Data.ACK_Received) {
#ifdef USART_debug
    USART_send("ACK_received from: ");
		USART_write_buf(fromNodeID, DEC);
		USART_send("\n");
#endif
		return true;
	}
  return false;
}

/**
  * @note to increase the chance of getting a packet across, call this function instead of send
  *       and it handles all the ACK requesting/retrying for you :)
  *       The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
  *       The reason for the semi-automaton is that the lib is interrupt driven and
  *       requires user action to read the received data and decide what to do with it
  *       replies usually take only 5..8ms at 50kbps@915MHz
  */
uint8_t RFM69W_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) 
{
	uint8_t i;
	uint32_t sentTime;
	
	RFM69W_listenModeOFF();
	RFM69W_setMode(RF69_MODE_STANDBY);
	
  for (i = 0; i <= retries; i++) {
    RFM69W_send(toAddress, buffer, bufferSize, 1);
		RFM69W_Data.ACK_Received = 0;
		// Enter to RX mode to get ACK
		RFM69W_setMode(RF69_MODE_RX);
		// Change interrupt source to PayloadReady
		RFM69W_setMap(DIO0, RF_DIOMAPPING1_DIO0_01);
		
    sentTime = SYS_TICK_timeOut(0, 0);
		
		while (SYS_TICK_timeOut(1, sentTime) < retryWaitTime) {
			// Waiting for ACK
			if (RFM69W_ACKReceived(toAddress)) {
				// ACK received- back to Standby
				RFM69W_setMode(RF69_MODE_STANDBY);
				return true;
			}
		}
		
		RFM69W_setMode(RF69_MODE_STANDBY);
	}
	
	// Sending failed- back to the receiving mode and return 0
	RFM69W_listenModeON();
	
	return false;
}

/**
  * @note 
  */
void RFM69W_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK)
{
	uint32_t timeOut;
	// Avoid RX deadlocks
	RFM69W_SPI_set_bit(REG_PACKETCONFIG2, RF_PACKET2_RXRESTART);
	
	timeOut = SYS_TICK_timeOut(0, 0);
	while (!RFM69W_canSend() && SYS_TICK_timeOut(1, timeOut) < RF69_CSMA_LIMIT_MS);
  
	RFM69W_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

/**
  * @note 
  */
uint8_t RFM69W_canSend(void)
{
	// If signal stronger than -100dBm is detected assume channel activity
	// Check if module is in RX mode- it means the module isnt in idle mode and it received some msg
	if (!((RFM69W_SPI_send_poll(RFM69W_read,  REG_OPMODE, 0x00) & RF_OPMODE_RECEIVER) == RF_OPMODE_RECEIVER) && RFM69W_readRSSI(false) < CSMA_LIMIT)
		return true;
	return false;
}

/**
  * @note Should be called immediately after reception in case sender wants ACK.
  */
void RFM69W_sendACK(const void* buffer, uint8_t bufferSize) 
{
	uint8_t sender = RFM69W_Data.SenderID;
	// Save payload received RSSI value
	int16_t _RSSI = RFM69W_Data.RSSI;
	
	// Avoid RX deadlocks
	RFM69W_SPI_set_bit(REG_PACKETCONFIG2, RF_PACKET2_RXRESTART);
	
	RFM69W_sendFrame(sender, buffer, bufferSize, false, true);
#ifdef USART_debug
	USART_send("ACK_sent to: ");
	USART_write_buf(sender, DEC);
	USART_send("\n");
#endif
  // Restore payload RSSI
	RFM69W_Data.RSSI = _RSSI; 
}

/**
  * @note Internal function
  */
uint8_t RFM69W_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK, uint8_t sendACK)
{
  uint8_t CTLbyte, cnt, result; 
	uint32_t txStart;
	
	// Turn off receiver to prevent reception while filling FIFO
	RFM69W_setMode(RF69_MODE_STANDBY);
	
	// DIO0 is "Packet Sent"
	RFM69W_setMap(DIO0, RF_DIOMAPPING1_DIO0_00);
	
  if (bufferSize > RF69_MAX_DATA_LEN)
		bufferSize = RF69_MAX_DATA_LEN;

	// Control byte
  if (sendACK)
    CTLbyte = 0x80;
  else if (requestACK)
    CTLbyte = 0x40;

	// Write to FIFO
	RFM69W_CS_DOWN;
	RFM69W_SPI_send_8poll(REG_FIFO | 0x80);
	RFM69W_SPI_send_8poll(bufferSize + 3);
	RFM69W_SPI_send_8poll(toAddress);
	RFM69W_SPI_send_8poll(RFM69W_Data.RFM69W_Address);
	RFM69W_SPI_send_8poll(CTLbyte);

  for (cnt = 0; cnt < bufferSize; cnt++)
		RFM69W_SPI_send_8poll((((uint8_t*) buffer)[cnt]));

	RFM69W_CS_UP;
	
  RFM69W_setMode(RF69_MODE_TX);
  txStart = SYS_TICK_timeOut(0, 0);
	
	// Wait for DIO0 to turn HIGH signalling transmission finish	
  while (PacketSent_Check && SYS_TICK_timeOut(1, txStart) < RF69_TX_LIMIT_MS); 
	
	// Checks if packet was sent correctly
	if (!PacketSent_Check) {
#ifdef USART_debug
	USART_send("Packet sent to: ");
	USART_write_buf(toAddress, DEC);
	USART_send(" from: ");
	USART_write_buf(RFM69W_Data.RFM69W_Address, DEC);
	USART_send(" with ");
	USART_write_buf(bufferSize, DEC);
	USART_send(" bytes. ");
	if (sendACK)
		USART_send("It sends ACK.\n");
	if (requestACK)
		USART_send("It requests ACK.\n");
#endif
		result = true;
	}
	else {
#ifdef USART_debug
	USART_send("Packet sending timed out.\n");
#endif
		result = false;
	}
	
  RFM69W_setMode(RF69_MODE_STANDBY);
	

	
	return result;
}

/**
  * @note 
  */
void EXTI15_10_IRQHandler(void)
{
	uint8_t mode;
	
	if (EXTI->PR & EXTI_PR_PR13) {
		mode = RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00);
		
		// Check if the interrupt is trigered by PacketSent (RF_OPMODE_TRANSMITTER)
		if ((mode & RF_OPMODE_MODEMASK) == RF_OPMODE_TRANSMITTER) {
#ifdef USART_debug
				USART_send("Packet sent.\n");
#endif
		}
		
		// Check if the interrupt is trigered by PayloadReady (RF_OPMODE_RECEIVER)
		if ((mode & RF_OPMODE_MODEMASK) == RF_OPMODE_RECEIVER) {
			// Turn listen mode off and enters standby mode
			RFM69W_listenModeOFF();
			
			// Clears all data
			RFM69W_receiveBegin();
			
			// Receives msg
			RFM69W_interruptHandler();
			
			// Handles data
			if (RFM69W_Data.ACK_Requested)
				RFM69W_sendACK(0, 0);
			
			if (!RFM69W_Data.ACK_Received) {
				MENU_PacketInterpreter();
#ifdef USART_debug
				USART_send("Interprates as:\n");
#endif
			}
			
			RFM69W_listenModeON();
		}
		
		EXTI->PR |= EXTI_PR_PR13;
	}
}

/**
  * @note Internal function - interrupt gets called when a packet is received
  */
void RFM69W_interruptHandler(void) 
{
	uint8_t CTLbyte;
	uint8_t i;
	
	RFM69W_CS_DOWN;
	RFM69W_SPI_send_8poll(REG_FIFO & 0x7F);
	RFM69W_Data.PayloadLen = RFM69W_SPI_send_8poll(0x00);
	RFM69W_Data.TargetID = RFM69W_SPI_send_8poll(0x00);
	RFM69W_Data.DataLen = RFM69W_Data.PayloadLen - 3;
	RFM69W_Data.SenderID = RFM69W_SPI_send_8poll(0x00);
	CTLbyte = RFM69W_SPI_send_8poll(0x00);

	// Extract ACK-received flag
	RFM69W_Data.ACK_Received = CTLbyte & 0x80;
	// Extract ACK-requested flag
	RFM69W_Data.ACK_Requested = CTLbyte & 0x40; 

	for (i = 0; i < RFM69W_Data.DataLen; i++)
		RFM69W_Data.Data[i] = RFM69W_SPI_send_8poll(0x00);
	
	// Add null at end of string
	if (RFM69W_Data.DataLen < RF69_MAX_DATA_LEN)
		RFM69W_Data.Data[RFM69W_Data.DataLen] = 0; 
	
	RFM69W_CS_UP;
	
  RFM69W_Data.RSSI = RFM69W_readRSSI(false);
	
#ifdef USART_debug
	USART_send("Packet received from: ");
	USART_write_buf(RFM69W_Data.SenderID, DEC);
	USART_send(" to: ");
	USART_write_buf(RFM69W_Data.TargetID, DEC);
	USART_send(" with ");
	USART_write_buf(RFM69W_Data.DataLen, DEC);
	USART_send(" bytes and ");
	USART_write_buf(RFM69W_Data.RSSI, DEC);
	USART_send(" RSSI. ");
	if (RFM69W_Data.ACK_Received)
		USART_send("ACK received.\n");
	if (RFM69W_Data.ACK_Requested)
		USART_send("It requests ACK.\n");
#endif
}

/**
  * @note Internal function
  */
void RFM69W_receiveBegin(void) 
{
	RFM69W_Data.DataLen = 0;
	RFM69W_Data.SenderID = 0;
	RFM69W_Data.TargetID = 0;
	RFM69W_Data.PayloadLen = 0;
	RFM69W_Data.ACK_Requested = 0;
	RFM69W_Data.ACK_Received = 0;
	RFM69W_Data.RSSI = 0;
  
	// Avoid RX deadlocks
	if (RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS2, 0x00) & RF_IRQFLAGS2_PAYLOADREADY)
		RFM69W_SPI_set_bit(REG_PACKETCONFIG2, RF_PACKET2_RXRESTART);
}

/**
  * @note To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
  *       To disable encryption: radio.encrypt(null) or radio.encrypt(0)
  *       KEY HAS TO BE 16 bytes !!!
  */
void RFM69W_encrypt(const char* key)
{
	uint8_t i;
	
	RFM69W_setMode(RF69_MODE_STANDBY);
	
  if (key != 0) {
		RFM69W_CS_DOWN;
		RFM69W_SPI_send_8poll(REG_AESKEY1 | 0x80);
		for (i = 0; i < 16; i++)
			RFM69W_SPI_send_8poll(key[i]);
		RFM69W_CS_UP;
		SPI1->CR1 &= ~SPI_CR1_SPE;
  }
	
	// Turns AES ON if key > 0
	RFM69W_SPI_set_bit(REG_PACKETCONFIG2, (key ? 1 : 0));
}

/**
  * @note Get the received signal strength indicator (RSSI)
  */
int16_t RFM69W_readRSSI(uint8_t forceTrigger)
{
  int16_t rssi = 0;
	
  if (forceTrigger) {
		// RSSI trigger not needed if DAGC is in continuous mode
		RFM69W_SPI_set_bit(REG_RSSICONFIG, RF_RSSI_START);
		// Wait for RSSI_Ready
    while ((RFM69W_SPI_send_poll(RFM69W_read, REG_RSSICONFIG, 0x00) & RF_RSSI_DONE) == 0x00); 
  }
	
  rssi = -RFM69W_SPI_send_poll(RFM69W_read, REG_RSSIVALUE, 0x00);
  rssi >>= 1;
	
  return rssi;
}

/**
  * @note 
  */
void RFM69W_readAllRegs(void)
{
  uint8_t regVal;
	uint8_t regAddr;

	USART_send("Add[dec]\tAdd_val[dec]\tAdd_val[bin]\n");

	
  for (regAddr = 1; regAddr <= 0x4F; regAddr++) {
		regVal = RFM69W_SPI_send_poll(RFM69W_read, regAddr, 0x00);
		
		USART_write_buf(regAddr, DEC);
		USART_send("\t\t");
    USART_write_buf(regVal, DEC);
    USART_send("\t\t");
    USART_write_buf(regVal, BIN);
		USART_send("\n");
  }
}

/**
  * @note 
  */
void RFM69W_listenModeON(void)
{
	RFM69W_setMode(RF69_MODE_STANDBY);
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_LISTEN_ON);
#ifdef USART_debug
	USART_send("Listen mode ON.\n");
#endif
}

void RFM69W_listenModeOFF(void)
{
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY);
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_STANDBY);
	
	while(!((RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & RF_OPMODE_MODEMASK) == RF_OPMODE_STANDBY)) {
		RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_STANDBY);
	}
#ifdef USART_debug
	USART_send("Listen mode OFF.\n");
#endif
}

void RFM69W_reset(void)
{
	uint32_t timeOut;
	// RFM69W reset sequence
	// GPIO config
	GPIO_config(0x0A, 9, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	// Pull high
	GPIOA->ODR |= GPIO_ODR_ODR_9;
	// Wait min 100 us (actually waits for 1 ms)
	timeOut = SYS_TICK_timeOut(0, 0);
	while (SYS_TICK_timeOut(1, timeOut) < 1);
	// Pull low
	GPIOA->ODR &= ~GPIO_ODR_ODR_9;
	// Wait min 5 ms
	timeOut = SYS_TICK_timeOut(0, 0);
	while (SYS_TICK_timeOut(1, timeOut) < 5);
	
#ifdef USART_debug
	USART_send("RFM69W reset done.\n");
#endif
}

/**
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  * @brief SPI and STM32L1 oriented functions  * 
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *
  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
  */

/**
  * @note 
  */

void RFM69W_SPI_set_bit(uint8_t ADDRESS, uint8_t BITS)
{
	RFM69W_SPI_send_poll(RFM69W_write, ADDRESS, RFM69W_SPI_send_poll(RFM69W_read, ADDRESS, 0x00) | BITS);
}

/**
  * @note 
  */

void RFM69W_SPI_clear_bit(uint8_t ADDRESS, uint8_t BITS)
{
	RFM69W_SPI_send_poll(RFM69W_write, ADDRESS, RFM69W_SPI_send_poll(RFM69W_read, ADDRESS, 0x00) & ~BITS);
}

/**
  * @note 
  */
uint16_t RFM69W_SPI_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{
	switch (TYPE) {
		case RFM69W_read:
			RFM69W_CS_DOWN;
			
			DATA = SPI1->DR;
			while(!(SPI1->SR & SPI_SR_TXE));
			while(SPI1->SR & SPI_SR_BSY);
			
			SPI1->DR = (ADDRESS << 8) | 0x00;
			
			while(!(SPI1->SR & SPI_SR_TXE));
			while(!(SPI1->SR & SPI_SR_RXNE));
			
			DATA = SPI1->DR;
			
			while(SPI1->SR & SPI_SR_BSY);
			
			RFM69W_CS_UP;
			
			return DATA;
		case RFM69W_write:
			RFM69W_CS_DOWN;
		
			while(!(SPI1->SR & SPI_SR_TXE));
			while(SPI1->SR & SPI_SR_BSY);
			
			SPI1->DR = ((ADDRESS | 0x80) << 8) | DATA;
			
			while(!(SPI1->SR & SPI_SR_TXE));
			while(SPI1->SR & SPI_SR_BSY);
			
			RFM69W_CS_UP;
			break;
	}
}

/**
  * @note 
  */
uint8_t RFM69W_SPI_send_8poll(uint8_t DATA)
{
	// 8 bit data frame
	SPI1->CR1 &= ~SPI_CR1_DFF;

	while(SPI1->SR & SPI_SR_BSY);
	SPI1->DR = DATA;
	
	while(!(SPI1->SR & SPI_SR_RXNE));
	DATA = SPI1->DR;
	
	while(SPI1->SR & SPI_SR_BSY);
	// Back to 16 bit data frame
	SPI1->CR1 |= SPI_CR1_DFF;
	
	return DATA;
}

/**
  * @note 
  */
void RFM69W_SPI_send(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
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
		while(SPI1->SR & SPI_SR_BSY);
		SPI1->DR = RFM69W_FIFO.buffer[RFM69W_FIFO.head].data;
	}
}

/**
  * @note 
  */
uint8_t RFM69W_SPI_write(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA) 
{
	// Check if FIFO is full
	if (RFM69W_FIFO.tail == (RFM69W_FIFO_size - 1)) {
		if (RFM69W_FIFO.head == 0) 
			return false;
	} else {
		if (RFM69W_FIFO.head == (RFM69W_FIFO.tail + 1))
			return false;
	}
	
	// If FIFO is not full, fill the TAIL buffer
	switch (TYPE) {
		case RFM69W_write:
			RFM69W_FIFO.buffer[RFM69W_FIFO.tail].type = RFM69W_write;
			RFM69W_FIFO.buffer[RFM69W_FIFO.tail].data = ((ADDRESS | 0x80) << 8 ) | DATA;
			break;
		case RFM69W_read:
			RFM69W_FIFO.buffer[RFM69W_FIFO.tail].type = RFM69W_read;
			RFM69W_FIFO.buffer[RFM69W_FIFO.tail].data = (ADDRESS << 8 ) | DATA;
			break;
	}
	
	// Increment TAIL
	if ((RFM69W_FIFO.tail + 1) == RFM69W_FIFO_size) {
		RFM69W_FIFO.tail = 0;
	} else {
		RFM69W_FIFO.tail++;
	}
	
	return true;
}

/**
  * @note 
  */
void SPI1_IRQHandler(void)
{
	uint16_t buffer;
	
	if (SPI1->SR & SPI_SR_RXNE) {
		buffer = SPI1->DR;
		RFM69W_CS_UP;
		
		if (RFM69W_FIFO.buffer[RFM69W_FIFO.head].type == RFM69W_read) {
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

/**
  * @note 
  */
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
	GPIO_config(0x0B, 4, GPIO_MODE_AF, GPIO_PULL_Pullup, GPIO_TYPE_Pushpull, GPIO_SPEED_2M, GPIO_AF_AF5);
	
	// CS configuration:
	GPIO_config(0x0A, 15, GPIO_MODE_GP, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_400k, 0);
	
	RFM69W_CS_UP;                                     // CS up
}

/**
  * @note 
  */
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
	
//	NVIC_EnableIRQ(SPI1_IRQn);
}

/**
  * @note 
  */
void RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
	RFM69W_REG_init();
}
