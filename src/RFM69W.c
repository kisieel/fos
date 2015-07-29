#include "RFM69W.h"
#include "USART.h"
#include "stm32l1xx.h"

// Available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define RF69_MAX_DATA_LEN      61
#define GATEWAYID              1
#define NODEID                 133
#define NETWORKID              100
#define FREQUENCY              RF69_868MHZ
#define ENCRYPTKEY             "FishingMonsters!" // Has to be same 16 characters/bytes on all nodes, not more not less!

#define COURSE_TEMP_COEF       -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR    255
#define RF69_CSMA_LIMIT_MS     1000
#define RF69_TX_LIMIT_MS       1000
#define RF69_FSTEP             61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

#define CSMA_LIMIT             -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

#define true                    1
#define false                   0

// Minimum size: 3
#define RFM69W_FIFO_size        20

#define RFM69W_CS_UP            GPIOA->ODR |= GPIO_ODR_ODR_15; SPI1->CR1 &= ~SPI_CR1_SPE; __enable_irq();
#define RFM69W_CS_DOWN          __disable_irq(); SPI1->CR1 |= SPI_CR1_SPE; GPIOA->ODR &= ~GPIO_ODR_ODR_15;

//#define RFM69W_write            1
//#define RFM69W_read             0

// Private functions

/**
  * @brief  RFM69W oriented functions       
  */
uint8_t  RFM69W_REG_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
// void EXTI15_10_IRQHandler(void);
uint32_t RFM69W_getFrequency(void);
void     RFM69W_setFrequency(uint32_t freqHz);
void     RFM69W_setMode(uint8_t newMode);
void     RFM69W_sleep(void);
void     RFM69W_setAddress(uint8_t addr);
void     RFM69W_setNetwork(uint8_t networkID);
void     RFM69W_setPowerLevel(uint8_t powerLevel);
uint8_t  RFM69W_canSend(void);
void     RFM69W_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK);
uint8_t  RFM69W_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);
uint8_t  RFM69W_ACKReceived(uint8_t fromNodeID);
uint8_t  RFM69W_ACKRequested(void);
void     RFM69W_sendACK(const void* buffer, uint8_t bufferSize);
void     RFM69W_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK, uint8_t sendACK);
void     RFM69W_interruptHandler(void);
void     RFM69W_receiveBegin(void);
uint8_t  RFM69W_receiveDone(void);
void     RFM69W_encrypt(const char* key);
int16_t  RFM69W_readRSSI(uint8_t forceTrigger);
void     RFM69W_promiscuous(uint8_t onOff);
void     RFM69W_setHighPower(uint8_t onOff);
void     RFM69W_setHighPowerRegs(uint8_t onOff);
void     RFM69W_readAllRegs(void);
void     RFM69W_rcCalibration(void);
void     RFM69W_listenModeON(void);
void     RFM69W_listenModeOFF(void);

uint32_t milis(uint8_t onOff);
	
/**
  * @brief  SPI and STM32L1 oriented functions       
  */

uint8_t  RFM69W_SPI_send_8poll(uint8_t DATA);
uint16_t RFM69W_SPI_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);
void     RFM69W_SPI_send(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);
uint8_t  RFM69W_SPI_write(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);

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

volatile uint8_t RFM69W_mode;
         uint8_t RFM69W_address;
         uint8_t RFM69W_promiscuousMode;

volatile uint8_t DATA[RF69_MAX_DATA_LEN];                // recv/xmit buf, including header & crc bytes
volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;                // should match _address
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED;            // should be polled immediately after sending a packet with ACK request
volatile int16_t RSSI;                    // most accurate RSSI during reception (closest to the reception)

volatile uint8_t RFM69W_powerLevel = 31;
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
uint8_t RFM69W_REG_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{	
	do
		RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE1, 0xAA);
	while
		(RFM69W_SPI_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0xAA);
	
	do
		RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE1, 0x55);
	while
		(RFM69W_SPI_send_poll(RFM69W_read, REG_SYNCVALUE1, 0x00) != 0x55);
	
	//
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
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
	// DIO0 is the only IRQ we're using
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
	// DIO5 ClkOut disable for power saving
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF);
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
	RFM69W_encrypt(0);
	
	// Called regardless if it's a RFM69W or RFM69HW
	RFM69W_setHighPower(0);
	RFM69W_setMode(RF69_MODE_STANDBY);
	
	// Wait for ModeReady
	while ((RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00);
  
	// RFM69_WAKE_uC as a DIO0 pin in RFM69W module
	// PA10 input floating in module
	GPIO_config(0x0A, 10, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;    // A block for EXTI10
	EXTI->IMR |= EXTI_IMR_MR10;                       // Input line 10 selection (unmasking)
	EXTI->RTSR |= EXTI_RTSR_TR10;                     // Rising edge selection
	NVIC_SetPriority(EXTI15_10_IRQn, 0);              // Priority set to 0
	NVIC_EnableIRQ(EXTI15_10_IRQn);                   // Interrupt enable
	
  RFM69W_address = NODEID;
	
	return true;
}

/**
  * @note Return the frequency (in Hz).
  */
uint32_t RFM69W_getFrequency(void)
{
  return RF69_FSTEP * (((uint32_t) RFM69W_SPI_send_poll(RFM69W_read, REG_FRFMSB, 0x00) << 16) + ((uint16_t) RFM69W_SPI_send_poll(RFM69W_read, REG_FRFMID, 0x00) << 8) + RFM69W_SPI_send_poll(RFM69W_read, REG_FRFLSB, 0x00));
}

/**
  * @note 
  */
void EXTI15_10_IRQHandler(void)
{
	if (EXTI->PR & EXTI_PR_PR10) {
		RFM69W_interruptHandler();
		EXTI->PR |= EXTI_PR_PR10;
	}
}



/**
  * @note Set the frequency (in Hz).
  */
void RFM69W_setFrequency(uint32_t freqHz)
{
  uint8_t oldMode = RFM69W_mode;
	
  if (oldMode == RF69_MODE_TX)
    RFM69W_setMode(RF69_MODE_RX);
	
	// Divide down by FSTEP to get FRF
  freqHz /= RF69_FSTEP;
   RFM69W_SPI_send_poll(RFM69W_write, REG_FRFMSB, freqHz >> 16);
   RFM69W_SPI_send_poll(RFM69W_write, REG_FRFMID, freqHz >> 8);
   RFM69W_SPI_send_poll(RFM69W_write, REG_FRFLSB, freqHz);
	
  if (oldMode == RF69_MODE_RX)
    RFM69W_setMode(RF69_MODE_SYNTH);
	
  RFM69W_setMode(oldMode);
}

/**
  * @note 
  */
void RFM69W_setMode(uint8_t newMode)
{
  if (newMode == RFM69W_mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, (RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, (RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, (RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, (RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
			RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, (RFM69W_SPI_send_poll(RFM69W_read, REG_OPMODE, 0x00) & 0xE3) | RF_OPMODE_SLEEP);
      break;
  }
	
	// We are using packet mode, so this check is not really needed but
  // waiting for mode ready is necessary when going from sleep because
	// the FIFO may not be immediately available from previous mode.
	// Wait for ModeReady.
	while (RFM69W_mode == RF69_MODE_SLEEP && (RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00);

  RFM69W_mode = newMode;
}

/**
  * @note Put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone().
  */
void RFM69W_sleep(void) 
{
  RFM69W_setMode(RF69_MODE_SLEEP);
}

/**
  * @note Set this node's address.
  */
void RFM69W_setAddress(uint8_t addr)
{
  RFM69W_address = addr;
  RFM69W_SPI_send_poll(RFM69W_write, REG_NODEADRS, RFM69W_address);
}

/**
  * @note Set this node's network id.
  */
void RFM69W_setNetwork(uint8_t networkID)
{
  RFM69W_SPI_send_poll(RFM69W_write, REG_SYNCVALUE2, networkID);
}

/**
  * @note set *transmit/TX* output power: 0=min, 31=max
  *       this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
  *       the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
  *       valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
  *       this function implements 2 modes as follows:
  *             - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
  *             - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
  */
void RFM69W_setPowerLevel(uint8_t powerLevel)
{
  RFM69W_powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  RFM69W_SPI_send_poll(RFM69W_write, REG_PALEVEL, (RFM69W_SPI_send_poll(RFM69W_read, REG_PALEVEL, 0x00) & 0xE0) | RFM69W_powerLevel);
}

/**
  * @note 
  */
uint8_t RFM69W_canSend(void)
{
	// If signal stronger than -100dBm is detected assume channel activity
  if (RFM69W_mode == RF69_MODE_RX && PAYLOADLEN == 0 && RFM69W_readRSSI(false) < CSMA_LIMIT) {
    RFM69W_setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

/**
  * @note 
  */
void RFM69W_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK)
{
	uint32_t now;
	// Avoid RX deadlocks
  RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, (RFM69W_SPI_send_poll(RFM69W_read, REG_PACKETCONFIG2, 0x00) & 0xFB) | RF_PACKET2_RXRESTART); 
	now = milis(1);
	while (!RFM69W_canSend() && milis(1) - now < RF69_CSMA_LIMIT_MS) 
		RFM69W_receiveDone();
	
	milis(0);
  
	RFM69W_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
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
  uint32_t sentTime;
	uint8_t i;
	
  for (i = 0; i <= retries; i++)
  {
    RFM69W_send(toAddress, buffer, bufferSize, 1);
    sentTime = milis(1);
    while (milis(1) - sentTime < retryWaitTime) {
      if (RFM69W_ACKReceived(toAddress))
        return true;
    }
  }
	
	milis(0);
	
  return false;
}

/**
  * @note Should be polled immediately after sending a packet with ACK request.
  */
uint8_t RFM69W_ACKReceived(uint8_t fromNodeID) 
{
  if (RFM69W_receiveDone())
    return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}

/**
  * @note Check whether an ACK was requested in the last received packet (non-broadcasted packet).
  */
uint8_t RFM69W_ACKRequested(void) 
{
  return ACK_REQUESTED && (TARGETID != RF69_BROADCAST_ADDR);
}

/**
  * @note Should be called immediately after reception in case sender wants ACK.
  */
void RFM69W_sendACK(const void* buffer, uint8_t bufferSize) 
{
	uint32_t now;
  uint8_t sender = SENDERID;
	// Save payload received RSSI value
  int16_t _RSSI = RSSI;
	
	// Avoid RX deadlocks
  RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, (RFM69W_SPI_send_poll(RFM69W_read, REG_PACKETCONFIG2, 0x00) & 0xFB) | RF_PACKET2_RXRESTART);
  now = milis(1);
  while (!RFM69W_canSend() && milis(1) - now < RF69_CSMA_LIMIT_MS) 
		RFM69W_receiveDone();
	milis(0);
  
	RFM69W_sendFrame(sender, buffer, bufferSize, false, true);
  // Restore payload RSSI
	RSSI = _RSSI; 
}

/**
  * @note Internal function
  */
void RFM69W_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK, uint8_t sendACK)
{
  uint8_t CTLbyte = 0x00; 
	uint32_t txStart;
	uint8_t i;
	
	// Turn off receiver to prevent reception while filling FIFO
  RFM69W_setMode(RF69_MODE_STANDBY);
	// Wait for ModeReady
  while ((RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00);
	// DIO0 is "Packet Sent"
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00);
	
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
	RFM69W_SPI_send_8poll(RFM69W_address);
	RFM69W_SPI_send_8poll(CTLbyte);

  for (i = 0; i < bufferSize; i++)
		RFM69W_SPI_send_8poll((((uint8_t*) buffer)[i]));

	RFM69W_CS_UP;
	
	// No need to wait for transmit mode to be ready since its handled by the radio
  RFM69W_setMode(RF69_MODE_TX);
  txStart = milis(1);
	// Wait for DIO0 to turn HIGH signalling transmission finish
  while ((GPIOA->IDR & GPIO_IDR_IDR_10) == 0 && milis(1) - txStart < RF69_TX_LIMIT_MS);
	milis(0);
	// wait for ModeReady
  // while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); 
  RFM69W_setMode(RF69_MODE_STANDBY);
}

/**
  * @note Internal function - interrupt gets called when a packet is received
  */
void RFM69W_interruptHandler(void) 
{
	uint8_t CTLbyte;
	uint8_t i;
	
  if (RFM69W_mode == RF69_MODE_RX && (RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS2, 0x00) & RF_IRQFLAGS2_PAYLOADREADY)) {
    //RSSI = readRSSI();
    RFM69W_setMode(RF69_MODE_STANDBY);
    
		RFM69W_CS_DOWN;
		RFM69W_SPI_send_8poll(REG_FIFO & 0x7F);
		PAYLOADLEN = RFM69W_SPI_send_8poll(0x00);
		PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; 
		TARGETID = RFM69W_SPI_send_8poll(0x00);
    
		// Match this node's address, or broadcast address or anything in promiscuous mode
		// Address situation could receive packets that are malformed and don't fit this libraries extra fields
		if(!(RFM69W_promiscuousMode || TARGETID == RFM69W_address || TARGETID == RF69_BROADCAST_ADDR) || PAYLOADLEN < 3) {
      PAYLOADLEN = 0;
      RFM69W_CS_UP;
      RFM69W_receiveBegin();
      return;
    }

    DATALEN = PAYLOADLEN - 3;
    SENDERID = RFM69W_SPI_send_8poll(0x00);
    CTLbyte = RFM69W_SPI_send_8poll(0x00);

		// Extract ACK-received flag
    ACK_RECEIVED = CTLbyte & 0x80;
		// Extract ACK-requested flag
    ACK_REQUESTED = CTLbyte & 0x40; 

    for (i = 0; i < DATALEN; i++)
      DATA[i] = RFM69W_SPI_send_8poll(0x00);
		
		// Add null at end of string
    if (DATALEN < RF69_MAX_DATA_LEN)
			DATA[DATALEN] = 0; 
    
		RFM69W_CS_UP;
    RFM69W_setMode(RF69_MODE_RX);
  }
  RSSI = RFM69W_readRSSI(false);
  //digitalWrite(4, 0);
}

/**
  * @note Internal function
  */
void RFM69W_receiveBegin(void) 
{
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RSSI = 0;
  
	// Avoid RX deadlocks
	if (RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS2, 0x00) & RF_IRQFLAGS2_PAYLOADREADY)
    RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, (RFM69W_SPI_send_poll(RFM69W_read, REG_PACKETCONFIG2, 0x00) & 0xFB) | RF_PACKET2_RXRESTART);
  
	// Set DIO0 to "PAYLOADREADY" in receive mode
	RFM69W_SPI_send_poll(RFM69W_write, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
  RFM69W_setMode(RF69_MODE_RX);
}

/**
  * @note Checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
  */
uint8_t RFM69W_receiveDone(void)
{
	// Re-enabled in unselect() via setMode() or via receiveBegin()
  __disable_irq();
  if (RFM69W_mode == RF69_MODE_RX && PAYLOADLEN > 0) {
		// Enables interrupts
    RFM69W_setMode(RF69_MODE_STANDBY); 
    return true;
  } else {
		// Already in RX no payload yet
		if (RFM69W_mode == RF69_MODE_RX) {
			// Explicitly re-enable interrupts
			__enable_irq(); 
			return false;
		}
	}
	
  RFM69W_receiveBegin();
  return false;
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
	
	RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG2, (RFM69W_SPI_send_poll(RFM69W_read, REG_PACKETCONFIG2, 0x00) & 0xFE) | (key ? 1 : 0));
}

/**
  * @note Get the received signal strength indicator (RSSI)
  */
int16_t RFM69W_readRSSI(uint8_t forceTrigger)
{
  int16_t rssi = 0;
	
  if (forceTrigger) {
		// RSSI trigger not needed if DAGC is in continuous mode
    RFM69W_SPI_send_poll(RFM69W_write, REG_RSSICONFIG, RF_RSSI_START);
		// Wait for RSSI_Ready
    while ((RFM69W_SPI_send_poll(RFM69W_read, REG_RSSICONFIG, 0x00) & RF_RSSI_DONE) == 0x00); 
  }
	
  rssi = -RFM69W_SPI_send_poll(RFM69W_read, REG_RSSIVALUE, 0x00);
  rssi >>= 1;
	
  return rssi;
}

/**
  * @note true  = disable filtering to capture all frames on network
  *       false = enable node/broadcast filtering to capture only frames sent to this/broadcast address
  */
void RFM69W_promiscuous(uint8_t onOff) {
  RFM69W_promiscuousMode = onOff;
//  RFM69W_SPI_send_poll(RFM69W_write, REG_PACKETCONFIG1, (RFM69W_SPI_send_poll(RFM69W_read, REG_PACKETCONFIG1, 0x00) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

/**
  * @note 
  */
void RFM69W_setHighPower(uint8_t _isRFM69HW)
{
  RFM69W_SPI_send_poll(RFM69W_write, REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    RFM69W_SPI_send_poll(RFM69W_write, REG_PALEVEL, (RFM69W_SPI_send_poll(RFM69W_read, REG_PALEVEL, 0x00) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    RFM69W_SPI_send_poll(RFM69W_write, REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RFM69W_powerLevel); // enable P0 only
}

/**
  * @note 
  */
void RFM69W_setHighPowerRegs(uint8_t onOff)
{
  RFM69W_SPI_send_poll(RFM69W_write, REG_TESTPA1, onOff ? 0x5D : 0x55);
  RFM69W_SPI_send_poll(RFM69W_write, REG_TESTPA2, onOff ? 0x7C : 0x70);
}

/**
  * @note 
  */
void RFM69W_readAllRegs(void)
{
  uint8_t regVal;
	uint8_t regAddr;

  for (regAddr = 1; regAddr <= 0x4F; regAddr++) {
		regVal = RFM69W_SPI_send_poll(RFM69W_read, regAddr, 0x00);
		
		USART_write_buf(regAddr, DEC);
		USART_send("\t");
    USART_write_buf(regVal, DEC);
    USART_send("\t");
    USART_write_buf(regVal, BIN);
		USART_send("\n");
  }
}

/**
  * @note 
  */
void RFM69W_rcCalibration(void)
{
  RFM69W_SPI_send_poll(RFM69W_write, REG_OSC1, RF_OSC1_RCCAL_START);
  while ((RFM69W_SPI_send_poll(RFM69W_read, REG_OSC1, 0x00) & RF_OSC1_RCCAL_DONE) == 0x00);
}

/**
  * @note Turns on the mili second timer or returns it's value
  */
uint32_t milis(uint8_t onOff)
{
	if (onOff) {
		if (TIM11->CR1 & TIM_CR1_CEN) {
			return TIM11->CNT;
		} else {
			RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
			TIM11->PSC = 32000 - 1;
			TIM11->ARR = 0xFFFFFFFF;
			TIM11->CR1 |= TIM_CR1_CEN;
			return 0;
		}
	} else {
		TIM11->CR1 &= ~TIM_CR1_CEN;
		return 1;
	}
}

/**
  * @note 
  */
void RFM69W_listenModeON(void)
{
	RFM69W_setMode(RF69_MODE_STANDBY);
	
	// RSSI and SyncAddressMatch criteria, 64 us RX resolution, 4.1 ms Idle resolution
	// End: Module stays in Rx mode. Listen mode stops and must be disabled (RFM69W_listenModeOFF must be called)
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN1, RF_LISTEN1_CRITERIA_RSSIANDSYNC | RF_LISTEN1_RESOL_RX_64 | RF_LISTEN1_RESOL_IDLE_4100 | RF_LISTEN1_END_00);
	// Idle time ~100 ms = 4.1 ms * 24
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN2, 24);
	// RX time ~10 ms = 64 us * 156
	RFM69W_SPI_send_poll(RFM69W_write, REG_LISTEN3, 156);
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_LISTEN_ON);
}

void RFM69W_listenModeOFF(void)
{
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_LISTENABORT | RF_OPMODE_RECEIVER);
	RFM69W_SPI_send_poll(RFM69W_write, REG_OPMODE, RF_OPMODE_RECEIVER);
	// Wait for ModeReady
	while ((RFM69W_SPI_send_poll(RFM69W_read, REG_IRQFLAGS1, 0x00) & RF_IRQFLAGS1_MODEREADY) == 0x00);
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
uint16_t RFM69W_SPI_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA)
{
	switch (TYPE) {
		case RFM69W_read:
			RFM69W_CS_DOWN;
			
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
	GPIO_config(0x0B, 4, GPIO_MODE_AF, GPIO_PULL_Floating, GPIO_TYPE_Pushpull, GPIO_SPEED_2M, GPIO_AF_AF5);
	
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
	
	NVIC_EnableIRQ(SPI1_IRQn);
}

/**
  * @note 
  */
void RFM69W_init(void)
{
	RFM69W_GPIO_init();
	RFM69W_SPI_init();
	RFM69W_REG_init(FREQUENCY, NODEID, NETWORKID);
}
