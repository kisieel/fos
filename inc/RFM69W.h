#ifndef __RFM69W_H
#define __RFM69W_H

#include <stdint.h>

//#define OUR_REG_DEFINE
#define LAB_REG_DEFINE

// Minimum size: 3
#define RFM69W_FIFO_size     20

#define RFM69W_write         1
#define RFM69W_read          0

// Public functions
void _RFM69W_init(void);
void _RFM69W_send(unsigned char TYPE, unsigned char ADDRESS, unsigned char DATA);
uint16_t _RFM69W_send_poll(uint8_t TYPE, uint8_t ADDRESS, uint8_t DATA);

void _RFM69W_setMode(uint8_t newMode);
// End of public functions


#ifdef LAB_REG_DEFINE

extern volatile uint8_t DATA[61];                // recv/xmit buf, including header & crc bytes
extern volatile uint8_t DATALEN;
extern volatile uint8_t SENDERID;
extern volatile uint8_t TARGETID;                // should match _address
extern volatile uint8_t PAYLOADLEN;
extern volatile uint8_t ACK_REQUESTED;
extern volatile uint8_t ACK_RECEIVED;            // should be polled immediately after sending a packet with ACK request
extern volatile int16_t RSSI;                    // most accurate RSSI during reception (closest to the reception)

extern volatile uint8_t _RFM69W_powerLevel;

#define GATEWAYID              1
#define NODEID                 133
#define NETWORKID              100
#define FREQUENCY              RF69_868MHZ
#define ENCRYPTKEY             "FishingMonsters!" // Has to be same 16 characters/bytes on all nodes, not more not less!

#define RF69_MODE_SLEEP   0 // XTAL OFF
#define RF69_MODE_STANDBY 1 // XTAL ON
#define RF69_MODE_SYNTH   2 // PLL ON
#define RF69_MODE_RX      3 // RX MODE
#define RF69_MODE_TX      4 // TX MODE

#define REG_FIFO          0x00
#define REG_OPMODE        0x01
#define REG_DATAMODUL     0x02
#define REG_BITRATEMSB    0x03
#define REG_BITRATELSB    0x04
#define REG_FDEVMSB       0x05
#define REG_FDEVLSB       0x06
#define REG_FRFMSB        0x07
#define REG_FRFMID        0x08
#define REG_FRFLSB        0x09
#define REG_OSC1          0x0A
#define REG_AFCCTRL       0x0B
#define REG_LOWBAT        0x0C
#define REG_LISTEN1       0x0D
#define REG_LISTEN2       0x0E
#define REG_LISTEN3       0x0F
#define REG_VERSION       0x10
#define REG_PALEVEL       0x11
#define REG_PARAMP        0x12
#define REG_OCP           0x13
#define REG_AGCREF        0x14  // not present on RFM69/SX1231
#define REG_AGCTHRESH1    0x15  // not present on RFM69/SX1231
#define REG_AGCTHRESH2    0x16  // not present on RFM69/SX1231
#define REG_AGCTHRESH3    0x17  // not present on RFM69/SX1231
#define REG_LNA           0x18
#define REG_RXBW          0x19
#define REG_AFCBW         0x1A
#define REG_OOKPEAK       0x1B
#define REG_OOKAVG        0x1C
#define REG_OOKFIX        0x1D
#define REG_AFCFEI        0x1E
#define REG_AFCMSB        0x1F
#define REG_AFCLSB        0x20
#define REG_FEIMSB        0x21
#define REG_FEILSB        0x22
#define REG_RSSICONFIG    0x23
#define REG_RSSIVALUE     0x24
#define REG_DIOMAPPING1   0x25
#define REG_DIOMAPPING2   0x26
#define REG_IRQFLAGS1     0x27
#define REG_IRQFLAGS2     0x28
#define REG_RSSITHRESH    0x29
#define REG_RXTIMEOUT1    0x2A
#define REG_RXTIMEOUT2    0x2B
#define REG_PREAMBLEMSB   0x2C
#define REG_PREAMBLELSB   0x2D
#define REG_SYNCCONFIG    0x2E
#define REG_SYNCVALUE1    0x2F
#define REG_SYNCVALUE2    0x30
#define REG_SYNCVALUE3    0x31
#define REG_SYNCVALUE4    0x32
#define REG_SYNCVALUE5    0x33
#define REG_SYNCVALUE6    0x34
#define REG_SYNCVALUE7    0x35
#define REG_SYNCVALUE8    0x36
#define REG_PACKETCONFIG1 0x37
#define REG_PAYLOADLENGTH 0x38
#define REG_NODEADRS      0x39
#define REG_BROADCASTADRS 0x3A
#define REG_AUTOMODES     0x3B
#define REG_FIFOTHRESH    0x3C
#define REG_PACKETCONFIG2 0x3D
#define REG_AESKEY1       0x3E
#define REG_AESKEY2       0x3F
#define REG_AESKEY3       0x40
#define REG_AESKEY4       0x41
#define REG_AESKEY5       0x42
#define REG_AESKEY6       0x43
#define REG_AESKEY7       0x44
#define REG_AESKEY8       0x45
#define REG_AESKEY9       0x46
#define REG_AESKEY10      0x47
#define REG_AESKEY11      0x48
#define REG_AESKEY12      0x49
#define REG_AESKEY13      0x4A
#define REG_AESKEY14      0x4B
#define REG_AESKEY15      0x4C
#define REG_AESKEY16      0x4D
#define REG_TEMP1         0x4E
#define REG_TEMP2         0x4F
#define REG_TESTLNA       0x58
#define REG_TESTPA1       0x5A  // only present on RFM69HW/SX1231H
#define REG_TESTPA2       0x5C  // only present on RFM69HW/SX1231H
#define REG_TESTDAGC      0x6F

//******************************************************
// RF69/SX1231 bit control definition
//******************************************************

// RegOpMode
#define RF_OPMODE_SEQUENCER_OFF       0x80
#define RF_OPMODE_SEQUENCER_ON        0x00  // Default

#define RF_OPMODE_LISTEN_ON           0x40
#define RF_OPMODE_LISTEN_OFF          0x00  // Default

#define RF_OPMODE_LISTENABORT         0x20

#define RF_OPMODE_SLEEP               0x00
#define RF_OPMODE_STANDBY             0x04  // Default
#define RF_OPMODE_SYNTHESIZER         0x08
#define RF_OPMODE_TRANSMITTER         0x0C
#define RF_OPMODE_RECEIVER            0x10


// RegDataModul
#define RF_DATAMODUL_DATAMODE_PACKET            0x00  // Default
#define RF_DATAMODUL_DATAMODE_CONTINUOUS        0x40
#define RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC 0x60

#define RF_DATAMODUL_MODULATIONTYPE_FSK         0x00  // Default
#define RF_DATAMODUL_MODULATIONTYPE_OOK         0x08

#define RF_DATAMODUL_MODULATIONSHAPING_00       0x00  // Default
#define RF_DATAMODUL_MODULATIONSHAPING_01       0x01
#define RF_DATAMODUL_MODULATIONSHAPING_10       0x02
#define RF_DATAMODUL_MODULATIONSHAPING_11       0x03


// RegBitRate (bits/sec) example bit rates
#define RF_BITRATEMSB_1200            0x68
#define RF_BITRATELSB_1200            0x2B
#define RF_BITRATEMSB_2400            0x34
#define RF_BITRATELSB_2400            0x15
#define RF_BITRATEMSB_4800            0x1A  // Default
#define RF_BITRATELSB_4800            0x0B  // Default
#define RF_BITRATEMSB_9600            0x0D
#define RF_BITRATELSB_9600            0x05
#define RF_BITRATEMSB_19200           0x06
#define RF_BITRATELSB_19200           0x83
#define RF_BITRATEMSB_38400           0x03
#define RF_BITRATELSB_38400           0x41

#define RF_BITRATEMSB_38323           0x03
#define RF_BITRATELSB_38323           0x43

#define RF_BITRATEMSB_34482           0x03
#define RF_BITRATELSB_34482           0xA0

#define RF_BITRATEMSB_76800           0x01
#define RF_BITRATELSB_76800           0xA1
#define RF_BITRATEMSB_153600          0x00
#define RF_BITRATELSB_153600          0xD0
#define RF_BITRATEMSB_57600           0x02
#define RF_BITRATELSB_57600           0x2C
#define RF_BITRATEMSB_115200          0x01
#define RF_BITRATELSB_115200          0x16
#define RF_BITRATEMSB_12500           0x0A
#define RF_BITRATELSB_12500           0x00
#define RF_BITRATEMSB_25000           0x05
#define RF_BITRATELSB_25000           0x00
#define RF_BITRATEMSB_50000           0x02
#define RF_BITRATELSB_50000           0x80
#define RF_BITRATEMSB_100000          0x01
#define RF_BITRATELSB_100000          0x40
#define RF_BITRATEMSB_150000          0x00
#define RF_BITRATELSB_150000          0xD5
#define RF_BITRATEMSB_200000          0x00
#define RF_BITRATELSB_200000          0xA0
#define RF_BITRATEMSB_250000          0x00
#define RF_BITRATELSB_250000          0x80
#define RF_BITRATEMSB_300000          0x00
#define RF_BITRATELSB_300000          0x6B
#define RF_BITRATEMSB_32768           0x03
#define RF_BITRATELSB_32768           0xD1
// custom bit rates
#define RF_BITRATEMSB_55555           0x02
#define RF_BITRATELSB_55555           0x40
#define RF_BITRATEMSB_200KBPS         0x00
#define RF_BITRATELSB_200KBPS         0xa0


// RegFdev - frequency deviation (Hz)
#define RF_FDEVMSB_2000             0x00
#define RF_FDEVLSB_2000             0x21
#define RF_FDEVMSB_5000             0x00  // Default
#define RF_FDEVLSB_5000             0x52  // Default
#define RF_FDEVMSB_7500             0x00
#define RF_FDEVLSB_7500             0x7B
#define RF_FDEVMSB_10000            0x00
#define RF_FDEVLSB_10000            0xA4
#define RF_FDEVMSB_15000            0x00
#define RF_FDEVLSB_15000            0xF6
#define RF_FDEVMSB_20000            0x01
#define RF_FDEVLSB_20000            0x48
#define RF_FDEVMSB_25000            0x01
#define RF_FDEVLSB_25000            0x9A
#define RF_FDEVMSB_30000            0x01
#define RF_FDEVLSB_30000            0xEC
#define RF_FDEVMSB_35000            0x02
#define RF_FDEVLSB_35000            0x3D
#define RF_FDEVMSB_40000            0x02
#define RF_FDEVLSB_40000            0x8F
#define RF_FDEVMSB_45000            0x02
#define RF_FDEVLSB_45000            0xE1
#define RF_FDEVMSB_50000            0x03
#define RF_FDEVLSB_50000            0x33
#define RF_FDEVMSB_55000            0x03
#define RF_FDEVLSB_55000            0x85
#define RF_FDEVMSB_60000            0x03
#define RF_FDEVLSB_60000            0xD7
#define RF_FDEVMSB_65000            0x04
#define RF_FDEVLSB_65000            0x29
#define RF_FDEVMSB_70000            0x04
#define RF_FDEVLSB_70000            0x7B
#define RF_FDEVMSB_75000            0x04
#define RF_FDEVLSB_75000            0xCD
#define RF_FDEVMSB_80000            0x05
#define RF_FDEVLSB_80000            0x1F
#define RF_FDEVMSB_85000            0x05
#define RF_FDEVLSB_85000            0x71
#define RF_FDEVMSB_90000            0x05
#define RF_FDEVLSB_90000            0xC3
#define RF_FDEVMSB_95000            0x06
#define RF_FDEVLSB_95000            0x14
#define RF_FDEVMSB_100000           0x06
#define RF_FDEVLSB_100000           0x66
#define RF_FDEVMSB_110000           0x07
#define RF_FDEVLSB_110000           0x0A
#define RF_FDEVMSB_120000           0x07
#define RF_FDEVLSB_120000           0xAE
#define RF_FDEVMSB_130000           0x08
#define RF_FDEVLSB_130000           0x52
#define RF_FDEVMSB_140000           0x08
#define RF_FDEVLSB_140000           0xF6
#define RF_FDEVMSB_150000           0x09
#define RF_FDEVLSB_150000           0x9A
#define RF_FDEVMSB_160000           0x0A
#define RF_FDEVLSB_160000           0x3D
#define RF_FDEVMSB_170000           0x0A
#define RF_FDEVLSB_170000           0xE1
#define RF_FDEVMSB_180000           0x0B
#define RF_FDEVLSB_180000           0x85
#define RF_FDEVMSB_190000           0x0C
#define RF_FDEVLSB_190000           0x29
#define RF_FDEVMSB_200000           0x0C
#define RF_FDEVLSB_200000           0xCD
#define RF_FDEVMSB_210000           0x0D
#define RF_FDEVLSB_210000           0x71
#define RF_FDEVMSB_220000           0x0E
#define RF_FDEVLSB_220000           0x14
#define RF_FDEVMSB_230000           0x0E
#define RF_FDEVLSB_230000           0xB8
#define RF_FDEVMSB_240000           0x0F
#define RF_FDEVLSB_240000           0x5C
#define RF_FDEVMSB_250000           0x10
#define RF_FDEVLSB_250000           0x00
#define RF_FDEVMSB_260000           0x10
#define RF_FDEVLSB_260000           0xA4
#define RF_FDEVMSB_270000           0x11
#define RF_FDEVLSB_270000           0x48
#define RF_FDEVMSB_280000           0x11
#define RF_FDEVLSB_280000           0xEC
#define RF_FDEVMSB_290000           0x12
#define RF_FDEVLSB_290000           0x8F
#define RF_FDEVMSB_300000           0x13
#define RF_FDEVLSB_300000           0x33


// RegFrf (MHz) - carrier frequency
// 315Mhz band
#define RF_FRFMSB_314             0x4E
#define RF_FRFMID_314             0x80
#define RF_FRFLSB_314             0x00
#define RF_FRFMSB_315             0x4E
#define RF_FRFMID_315             0xC0
#define RF_FRFLSB_315             0x00
#define RF_FRFMSB_316             0x4F
#define RF_FRFMID_316             0x00
#define RF_FRFLSB_316             0x00
// 433mhz band
#define RF_FRFMSB_433             0x6C
#define RF_FRFMID_433             0x40
#define RF_FRFLSB_433             0x00
#define RF_FRFMSB_434             0x6C
#define RF_FRFMID_434             0x80
#define RF_FRFLSB_434             0x00
#define RF_FRFMSB_435             0x6C
#define RF_FRFMID_435             0xC0
#define RF_FRFLSB_435             0x00
// 868Mhz band
#define RF_FRFMSB_863             0xD7
#define RF_FRFMID_863             0xC0
#define RF_FRFLSB_863             0x00
#define RF_FRFMSB_864             0xD8
#define RF_FRFMID_864             0x00
#define RF_FRFLSB_864             0x00
#define RF_FRFMSB_865             0xD8
#define RF_FRFMID_865             0x40
#define RF_FRFLSB_865             0x00
#define RF_FRFMSB_866             0xD8
#define RF_FRFMID_866             0x80
#define RF_FRFLSB_866             0x00
#define RF_FRFMSB_867             0xD8
#define RF_FRFMID_867             0xC0
#define RF_FRFLSB_867             0x00
#define RF_FRFMSB_868             0xD9
#define RF_FRFMID_868             0x00
#define RF_FRFLSB_868             0x00
#define RF_FRFMSB_869             0xD9
#define RF_FRFMID_869             0x40
#define RF_FRFLSB_869             0x00
#define RF_FRFMSB_870             0xD9
#define RF_FRFMID_870             0x80
#define RF_FRFLSB_870             0x00
// 915Mhz band
#define RF_FRFMSB_902             0xE1
#define RF_FRFMID_902             0x80
#define RF_FRFLSB_902             0x00
#define RF_FRFMSB_903             0xE1
#define RF_FRFMID_903             0xC0
#define RF_FRFLSB_903             0x00
#define RF_FRFMSB_904             0xE2
#define RF_FRFMID_904             0x00
#define RF_FRFLSB_904             0x00
#define RF_FRFMSB_905             0xE2
#define RF_FRFMID_905             0x40
#define RF_FRFLSB_905             0x00
#define RF_FRFMSB_906             0xE2
#define RF_FRFMID_906             0x80
#define RF_FRFLSB_906             0x00
#define RF_FRFMSB_907             0xE2
#define RF_FRFMID_907             0xC0
#define RF_FRFLSB_907             0x00
#define RF_FRFMSB_908             0xE3
#define RF_FRFMID_908             0x00
#define RF_FRFLSB_908             0x00
#define RF_FRFMSB_909             0xE3
#define RF_FRFMID_909             0x40
#define RF_FRFLSB_909             0x00
#define RF_FRFMSB_910             0xE3
#define RF_FRFMID_910             0x80
#define RF_FRFLSB_910             0x00
#define RF_FRFMSB_911             0xE3
#define RF_FRFMID_911             0xC0
#define RF_FRFLSB_911             0x00
#define RF_FRFMSB_912             0xE4
#define RF_FRFMID_912             0x00
#define RF_FRFLSB_912             0x00
#define RF_FRFMSB_913             0xE4
#define RF_FRFMID_913             0x40
#define RF_FRFLSB_913             0x00
#define RF_FRFMSB_914             0xE4
#define RF_FRFMID_914             0x80
#define RF_FRFLSB_914             0x00
#define RF_FRFMSB_915             0xE4  // Default
#define RF_FRFMID_915             0xC0  // Default
#define RF_FRFLSB_915             0x00  // Default
#define RF_FRFMSB_916             0xE5
#define RF_FRFMID_916             0x00
#define RF_FRFLSB_916             0x00
#define RF_FRFMSB_917             0xE5
#define RF_FRFMID_917             0x40
#define RF_FRFLSB_917             0x00
#define RF_FRFMSB_918             0xE5
#define RF_FRFMID_918             0x80
#define RF_FRFLSB_918             0x00
#define RF_FRFMSB_919             0xE5
#define RF_FRFMID_919             0xC0
#define RF_FRFLSB_919             0x00
#define RF_FRFMSB_920             0xE6
#define RF_FRFMID_920             0x00
#define RF_FRFLSB_920             0x00
#define RF_FRFMSB_921             0xE6
#define RF_FRFMID_921             0x40
#define RF_FRFLSB_921             0x00
#define RF_FRFMSB_922             0xE6
#define RF_FRFMID_922             0x80
#define RF_FRFLSB_922             0x00
#define RF_FRFMSB_923             0xE6
#define RF_FRFMID_923             0xC0
#define RF_FRFLSB_923             0x00
#define RF_FRFMSB_924             0xE7
#define RF_FRFMID_924             0x00
#define RF_FRFLSB_924             0x00
#define RF_FRFMSB_925             0xE7
#define RF_FRFMID_925             0x40
#define RF_FRFLSB_925             0x00
#define RF_FRFMSB_926             0xE7
#define RF_FRFMID_926             0x80
#define RF_FRFLSB_926             0x00
#define RF_FRFMSB_927             0xE7
#define RF_FRFMID_927             0xC0
#define RF_FRFLSB_927             0x00
#define RF_FRFMSB_928             0xE8
#define RF_FRFMID_928             0x00
#define RF_FRFLSB_928             0x00


// RegOsc1
#define RF_OSC1_RCCAL_START       0x80
#define RF_OSC1_RCCAL_DONE        0x40


// RegAfcCtrl
#define RF_AFCCTRL_LOWBETA_OFF    0x00  // Default
#define RF_AFCCTRL_LOWBETA_ON     0x20


// RegLowBat
#define RF_LOWBAT_MONITOR         0x10
#define RF_LOWBAT_ON              0x08
#define RF_LOWBAT_OFF             0x00  // Default

#define RF_LOWBAT_TRIM_1695       0x00
#define RF_LOWBAT_TRIM_1764       0x01
#define RF_LOWBAT_TRIM_1835       0x02  // Default
#define RF_LOWBAT_TRIM_1905       0x03
#define RF_LOWBAT_TRIM_1976       0x04
#define RF_LOWBAT_TRIM_2045       0x05
#define RF_LOWBAT_TRIM_2116       0x06
#define RF_LOWBAT_TRIM_2185       0x07


// RegListen1
#define RF_LISTEN1_RESOL_64       0x50
#define RF_LISTEN1_RESOL_4100     0xA0  // Default
#define RF_LISTEN1_RESOL_262000   0xF0

#define RF_LISTEN1_RESOL_IDLE_64     0x40
#define RF_LISTEN1_RESOL_IDLE_4100   0x80  // Default
#define RF_LISTEN1_RESOL_IDLE_262000 0xC0

#define RF_LISTEN1_RESOL_RX_64       0x10
#define RF_LISTEN1_RESOL_RX_4100     0x20  // Default
#define RF_LISTEN1_RESOL_RX_262000   0x30

#define RF_LISTEN1_CRITERIA_RSSI          0x00  // Default
#define RF_LISTEN1_CRITERIA_RSSIANDSYNC   0x08

#define RF_LISTEN1_END_00                 0x00
#define RF_LISTEN1_END_01                 0x02  // Default
#define RF_LISTEN1_END_10                 0x04


// RegListen2
#define RF_LISTEN2_COEFIDLE_VALUE         0xF5 // Default


// RegListen3
#define RF_LISTEN3_COEFRX_VALUE           0x20 // Default


// RegVersion
#define RF_VERSION_VER        0x24  // Default


// RegPaLevel
#define RF_PALEVEL_PA0_ON     0x80  // Default
#define RF_PALEVEL_PA0_OFF    0x00
#define RF_PALEVEL_PA1_ON     0x40
#define RF_PALEVEL_PA1_OFF    0x00  // Default
#define RF_PALEVEL_PA2_ON     0x20
#define RF_PALEVEL_PA2_OFF    0x00  // Default

#define RF_PALEVEL_OUTPUTPOWER_00000      0x00
#define RF_PALEVEL_OUTPUTPOWER_00001      0x01
#define RF_PALEVEL_OUTPUTPOWER_00010      0x02
#define RF_PALEVEL_OUTPUTPOWER_00011      0x03
#define RF_PALEVEL_OUTPUTPOWER_00100      0x04
#define RF_PALEVEL_OUTPUTPOWER_00101      0x05
#define RF_PALEVEL_OUTPUTPOWER_00110      0x06
#define RF_PALEVEL_OUTPUTPOWER_00111      0x07
#define RF_PALEVEL_OUTPUTPOWER_01000      0x08
#define RF_PALEVEL_OUTPUTPOWER_01001      0x09
#define RF_PALEVEL_OUTPUTPOWER_01010      0x0A
#define RF_PALEVEL_OUTPUTPOWER_01011      0x0B
#define RF_PALEVEL_OUTPUTPOWER_01100      0x0C
#define RF_PALEVEL_OUTPUTPOWER_01101      0x0D
#define RF_PALEVEL_OUTPUTPOWER_01110      0x0E
#define RF_PALEVEL_OUTPUTPOWER_01111      0x0F
#define RF_PALEVEL_OUTPUTPOWER_10000      0x10
#define RF_PALEVEL_OUTPUTPOWER_10001      0x11
#define RF_PALEVEL_OUTPUTPOWER_10010      0x12
#define RF_PALEVEL_OUTPUTPOWER_10011      0x13
#define RF_PALEVEL_OUTPUTPOWER_10100      0x14
#define RF_PALEVEL_OUTPUTPOWER_10101      0x15
#define RF_PALEVEL_OUTPUTPOWER_10110      0x16
#define RF_PALEVEL_OUTPUTPOWER_10111      0x17
#define RF_PALEVEL_OUTPUTPOWER_11000      0x18
#define RF_PALEVEL_OUTPUTPOWER_11001      0x19
#define RF_PALEVEL_OUTPUTPOWER_11010      0x1A
#define RF_PALEVEL_OUTPUTPOWER_11011      0x1B
#define RF_PALEVEL_OUTPUTPOWER_11100      0x1C
#define RF_PALEVEL_OUTPUTPOWER_11101      0x1D
#define RF_PALEVEL_OUTPUTPOWER_11110      0x1E
#define RF_PALEVEL_OUTPUTPOWER_11111      0x1F  // Default


// RegPaRamp
#define RF_PARAMP_3400            0x00
#define RF_PARAMP_2000            0x01
#define RF_PARAMP_1000            0x02
#define RF_PARAMP_500             0x03
#define RF_PARAMP_250             0x04
#define RF_PARAMP_125             0x05
#define RF_PARAMP_100             0x06
#define RF_PARAMP_62              0x07
#define RF_PARAMP_50              0x08
#define RF_PARAMP_40              0x09  // Default
#define RF_PARAMP_31              0x0A
#define RF_PARAMP_25              0x0B
#define RF_PARAMP_20              0x0C
#define RF_PARAMP_15              0x0D
#define RF_PARAMP_12              0x0E
#define RF_PARAMP_10              0x0F


// RegOcp
#define RF_OCP_OFF                0x0F
#define RF_OCP_ON                 0x1A  // Default

#define RF_OCP_TRIM_45            0x00
#define RF_OCP_TRIM_50            0x01
#define RF_OCP_TRIM_55            0x02
#define RF_OCP_TRIM_60            0x03
#define RF_OCP_TRIM_65            0x04
#define RF_OCP_TRIM_70            0x05
#define RF_OCP_TRIM_75            0x06
#define RF_OCP_TRIM_80            0x07
#define RF_OCP_TRIM_85            0x08
#define RF_OCP_TRIM_90            0x09
#define RF_OCP_TRIM_95            0x0A  // Default
#define RF_OCP_TRIM_100           0x0B
#define RF_OCP_TRIM_105           0x0C
#define RF_OCP_TRIM_110           0x0D
#define RF_OCP_TRIM_115           0x0E
#define RF_OCP_TRIM_120           0x0F


// RegAgcRef - not present on RFM69/SX1231
#define RF_AGCREF_AUTO_ON         0x40  // Default
#define RF_AGCREF_AUTO_OFF        0x00

#define RF_AGCREF_LEVEL_MINUS80   0x00  // Default
#define RF_AGCREF_LEVEL_MINUS81   0x01
#define RF_AGCREF_LEVEL_MINUS82   0x02
#define RF_AGCREF_LEVEL_MINUS83   0x03
#define RF_AGCREF_LEVEL_MINUS84   0x04
#define RF_AGCREF_LEVEL_MINUS85   0x05
#define RF_AGCREF_LEVEL_MINUS86   0x06
#define RF_AGCREF_LEVEL_MINUS87   0x07
#define RF_AGCREF_LEVEL_MINUS88   0x08
#define RF_AGCREF_LEVEL_MINUS89   0x09
#define RF_AGCREF_LEVEL_MINUS90   0x0A
#define RF_AGCREF_LEVEL_MINUS91   0x0B
#define RF_AGCREF_LEVEL_MINUS92   0x0C
#define RF_AGCREF_LEVEL_MINUS93   0x0D
#define RF_AGCREF_LEVEL_MINUS94   0x0E
#define RF_AGCREF_LEVEL_MINUS95   0x0F
#define RF_AGCREF_LEVEL_MINUS96   0x10
#define RF_AGCREF_LEVEL_MINUS97   0x11
#define RF_AGCREF_LEVEL_MINUS98   0x12
#define RF_AGCREF_LEVEL_MINUS99   0x13
#define RF_AGCREF_LEVEL_MINUS100  0x14
#define RF_AGCREF_LEVEL_MINUS101  0x15
#define RF_AGCREF_LEVEL_MINUS102  0x16
#define RF_AGCREF_LEVEL_MINUS103  0x17
#define RF_AGCREF_LEVEL_MINUS104  0x18
#define RF_AGCREF_LEVEL_MINUS105  0x19
#define RF_AGCREF_LEVEL_MINUS106  0x1A
#define RF_AGCREF_LEVEL_MINUS107  0x1B
#define RF_AGCREF_LEVEL_MINUS108  0x1C
#define RF_AGCREF_LEVEL_MINUS109  0x1D
#define RF_AGCREF_LEVEL_MINUS110  0x1E
#define RF_AGCREF_LEVEL_MINUS111  0x1F
#define RF_AGCREF_LEVEL_MINUS112  0x20
#define RF_AGCREF_LEVEL_MINUS113  0x21
#define RF_AGCREF_LEVEL_MINUS114  0x22
#define RF_AGCREF_LEVEL_MINUS115  0x23
#define RF_AGCREF_LEVEL_MINUS116  0x24
#define RF_AGCREF_LEVEL_MINUS117  0x25
#define RF_AGCREF_LEVEL_MINUS118  0x26
#define RF_AGCREF_LEVEL_MINUS119  0x27
#define RF_AGCREF_LEVEL_MINUS120  0x28
#define RF_AGCREF_LEVEL_MINUS121  0x29
#define RF_AGCREF_LEVEL_MINUS122  0x2A
#define RF_AGCREF_LEVEL_MINUS123  0x2B
#define RF_AGCREF_LEVEL_MINUS124  0x2C
#define RF_AGCREF_LEVEL_MINUS125  0x2D
#define RF_AGCREF_LEVEL_MINUS126  0x2E
#define RF_AGCREF_LEVEL_MINUS127  0x2F
#define RF_AGCREF_LEVEL_MINUS128  0x30
#define RF_AGCREF_LEVEL_MINUS129  0x31
#define RF_AGCREF_LEVEL_MINUS130  0x32
#define RF_AGCREF_LEVEL_MINUS131  0x33
#define RF_AGCREF_LEVEL_MINUS132  0x34
#define RF_AGCREF_LEVEL_MINUS133  0x35
#define RF_AGCREF_LEVEL_MINUS134  0x36
#define RF_AGCREF_LEVEL_MINUS135  0x37
#define RF_AGCREF_LEVEL_MINUS136  0x38
#define RF_AGCREF_LEVEL_MINUS137  0x39
#define RF_AGCREF_LEVEL_MINUS138  0x3A
#define RF_AGCREF_LEVEL_MINUS139  0x3B
#define RF_AGCREF_LEVEL_MINUS140  0x3C
#define RF_AGCREF_LEVEL_MINUS141  0x3D
#define RF_AGCREF_LEVEL_MINUS142  0x3E
#define RF_AGCREF_LEVEL_MINUS143  0x3F


// RegAgcThresh1 - not present on RFM69/SX1231
#define RF_AGCTHRESH1_SNRMARGIN_000   0x00
#define RF_AGCTHRESH1_SNRMARGIN_001   0x20
#define RF_AGCTHRESH1_SNRMARGIN_010   0x40
#define RF_AGCTHRESH1_SNRMARGIN_011   0x60
#define RF_AGCTHRESH1_SNRMARGIN_100   0x80
#define RF_AGCTHRESH1_SNRMARGIN_101   0xA0  // Default
#define RF_AGCTHRESH1_SNRMARGIN_110   0xC0
#define RF_AGCTHRESH1_SNRMARGIN_111   0xE0

#define RF_AGCTHRESH1_STEP1_0         0x00
#define RF_AGCTHRESH1_STEP1_1         0x01
#define RF_AGCTHRESH1_STEP1_2         0x02
#define RF_AGCTHRESH1_STEP1_3         0x03
#define RF_AGCTHRESH1_STEP1_4         0x04
#define RF_AGCTHRESH1_STEP1_5         0x05
#define RF_AGCTHRESH1_STEP1_6         0x06
#define RF_AGCTHRESH1_STEP1_7         0x07
#define RF_AGCTHRESH1_STEP1_8         0x08
#define RF_AGCTHRESH1_STEP1_9         0x09
#define RF_AGCTHRESH1_STEP1_10        0x0A
#define RF_AGCTHRESH1_STEP1_11        0x0B
#define RF_AGCTHRESH1_STEP1_12        0x0C
#define RF_AGCTHRESH1_STEP1_13        0x0D
#define RF_AGCTHRESH1_STEP1_14        0x0E
#define RF_AGCTHRESH1_STEP1_15        0x0F
#define RF_AGCTHRESH1_STEP1_16        0x10  // Default
#define RF_AGCTHRESH1_STEP1_17        0x11
#define RF_AGCTHRESH1_STEP1_18        0x12
#define RF_AGCTHRESH1_STEP1_19        0x13
#define RF_AGCTHRESH1_STEP1_20        0x14
#define RF_AGCTHRESH1_STEP1_21        0x15
#define RF_AGCTHRESH1_STEP1_22        0x16
#define RF_AGCTHRESH1_STEP1_23        0x17
#define RF_AGCTHRESH1_STEP1_24        0x18
#define RF_AGCTHRESH1_STEP1_25        0x19
#define RF_AGCTHRESH1_STEP1_26        0x1A
#define RF_AGCTHRESH1_STEP1_27        0x1B
#define RF_AGCTHRESH1_STEP1_28        0x1C
#define RF_AGCTHRESH1_STEP1_29        0x1D
#define RF_AGCTHRESH1_STEP1_30        0x1E
#define RF_AGCTHRESH1_STEP1_31        0x1F


// RegAgcThresh2 - not present on RFM69/SX1231
#define RF_AGCTHRESH2_STEP2_0         0x00
#define RF_AGCTHRESH2_STEP2_1         0x10
#define RF_AGCTHRESH2_STEP2_2         0x20
#define RF_AGCTHRESH2_STEP2_3         0x30  // XXX wrong -- Default
#define RF_AGCTHRESH2_STEP2_4         0x40
#define RF_AGCTHRESH2_STEP2_5         0x50
#define RF_AGCTHRESH2_STEP2_6         0x60
#define RF_AGCTHRESH2_STEP2_7         0x70  // default
#define RF_AGCTHRESH2_STEP2_8         0x80
#define RF_AGCTHRESH2_STEP2_9         0x90
#define RF_AGCTHRESH2_STEP2_10        0xA0
#define RF_AGCTHRESH2_STEP2_11        0xB0
#define RF_AGCTHRESH2_STEP2_12        0xC0
#define RF_AGCTHRESH2_STEP2_13        0xD0
#define RF_AGCTHRESH2_STEP2_14        0xE0
#define RF_AGCTHRESH2_STEP2_15        0xF0

#define RF_AGCTHRESH2_STEP3_0         0x00
#define RF_AGCTHRESH2_STEP3_1         0x01
#define RF_AGCTHRESH2_STEP3_2         0x02
#define RF_AGCTHRESH2_STEP3_3         0x03
#define RF_AGCTHRESH2_STEP3_4         0x04
#define RF_AGCTHRESH2_STEP3_5         0x05
#define RF_AGCTHRESH2_STEP3_6         0x06
#define RF_AGCTHRESH2_STEP3_7         0x07
#define RF_AGCTHRESH2_STEP3_8         0x08
#define RF_AGCTHRESH2_STEP3_9         0x09
#define RF_AGCTHRESH2_STEP3_10        0x0A
#define RF_AGCTHRESH2_STEP3_11        0x0B  // Default
#define RF_AGCTHRESH2_STEP3_12        0x0C
#define RF_AGCTHRESH2_STEP3_13        0x0D
#define RF_AGCTHRESH2_STEP3_14        0x0E
#define RF_AGCTHRESH2_STEP3_15        0x0F


// RegAgcThresh3 - not present on RFM69/SX1231
#define RF_AGCTHRESH3_STEP4_0         0x00
#define RF_AGCTHRESH3_STEP4_1         0x10
#define RF_AGCTHRESH3_STEP4_2         0x20
#define RF_AGCTHRESH3_STEP4_3         0x30
#define RF_AGCTHRESH3_STEP4_4         0x40
#define RF_AGCTHRESH3_STEP4_5         0x50
#define RF_AGCTHRESH3_STEP4_6         0x60
#define RF_AGCTHRESH3_STEP4_7         0x70
#define RF_AGCTHRESH3_STEP4_8         0x80
#define RF_AGCTHRESH3_STEP4_9         0x90  // Default
#define RF_AGCTHRESH3_STEP4_10        0xA0
#define RF_AGCTHRESH3_STEP4_11        0xB0
#define RF_AGCTHRESH3_STEP4_12        0xC0
#define RF_AGCTHRESH3_STEP4_13        0xD0
#define RF_AGCTHRESH3_STEP4_14        0xE0
#define RF_AGCTHRESH3_STEP4_15        0xF0

#define RF_AGCTHRESH3_STEP5_0         0x00
#define RF_AGCTHRESH3_STEP5_1         0x01
#define RF_AGCTHRESH3_STEP5_2         0x02
#define RF_AGCTHRESH3_STEP5_3         0x03
#define RF_AGCTHRESH3_STEP5_4         0x04
#define RF_AGCTHRESH3_STEP5_5         0x05
#define RF_AGCTHRESH3_STEP5_6         0x06
#define RF_AGCTHRESH3_STEP5_7         0x07
#define RF_AGCTHRES33_STEP5_8         0x08
#define RF_AGCTHRESH3_STEP5_9         0x09
#define RF_AGCTHRESH3_STEP5_10        0x0A
#define RF_AGCTHRESH3_STEP5_11        0x0B  // Default
#define RF_AGCTHRESH3_STEP5_12        0x0C
#define RF_AGCTHRESH3_STEP5_13        0x0D
#define RF_AGCTHRESH3_STEP5_14        0x0E
#define RF_AGCTHRESH3_STEP5_15        0x0F


// RegLna
#define RF_LNA_ZIN_50                 0x00  // Reset value
#define RF_LNA_ZIN_200                0x80  // Recommended default

#define RF_LNA_LOWPOWER_OFF           0x00  // Default
#define RF_LNA_LOWPOWER_ON            0x40

#define RF_LNA_CURRENTGAIN            0x08

#define RF_LNA_GAINSELECT_AUTO        0x00  // Default
#define RF_LNA_GAINSELECT_MAX         0x01
#define RF_LNA_GAINSELECT_MAXMINUS6   0x02
#define RF_LNA_GAINSELECT_MAXMINUS12  0x03
#define RF_LNA_GAINSELECT_MAXMINUS24  0x04
#define RF_LNA_GAINSELECT_MAXMINUS36  0x05
#define RF_LNA_GAINSELECT_MAXMINUS48  0x06


// RegRxBw
#define RF_RXBW_DCCFREQ_000           0x00
#define RF_RXBW_DCCFREQ_001           0x20
#define RF_RXBW_DCCFREQ_010           0x40  // Recommended default
#define RF_RXBW_DCCFREQ_011           0x60
#define RF_RXBW_DCCFREQ_100           0x80  // Reset value
#define RF_RXBW_DCCFREQ_101           0xA0
#define RF_RXBW_DCCFREQ_110           0xC0
#define RF_RXBW_DCCFREQ_111           0xE0

#define RF_RXBW_MANT_16               0x00  // Reset value
#define RF_RXBW_MANT_20               0x08
#define RF_RXBW_MANT_24               0x10  // Recommended default

#define RF_RXBW_EXP_0                 0x00
#define RF_RXBW_EXP_1                 0x01
#define RF_RXBW_EXP_2                 0x02
#define RF_RXBW_EXP_3                 0x03
#define RF_RXBW_EXP_4                 0x04
#define RF_RXBW_EXP_5                 0x05  // Recommended default
#define RF_RXBW_EXP_6                 0x06  // Reset value
#define RF_RXBW_EXP_7                 0x07


// RegAfcBw
#define RF_AFCBW_DCCFREQAFC_000       0x00
#define RF_AFCBW_DCCFREQAFC_001       0x20
#define RF_AFCBW_DCCFREQAFC_010       0x40
#define RF_AFCBW_DCCFREQAFC_011       0x60
#define RF_AFCBW_DCCFREQAFC_100       0x80  // Default
#define RF_AFCBW_DCCFREQAFC_101       0xA0
#define RF_AFCBW_DCCFREQAFC_110       0xC0
#define RF_AFCBW_DCCFREQAFC_111       0xE0

#define RF_AFCBW_MANTAFC_16           0x00
#define RF_AFCBW_MANTAFC_20           0x08  // Default
#define RF_AFCBW_MANTAFC_24           0x10

#define RF_AFCBW_EXPAFC_0             0x00
#define RF_AFCBW_EXPAFC_1             0x01
#define RF_AFCBW_EXPAFC_2             0x02  // Reset value
#define RF_AFCBW_EXPAFC_3             0x03  // Recommended default
#define RF_AFCBW_EXPAFC_4             0x04
#define RF_AFCBW_EXPAFC_5             0x05
#define RF_AFCBW_EXPAFC_6             0x06
#define RF_AFCBW_EXPAFC_7             0x07


// RegOokPeak
#define RF_OOKPEAK_THRESHTYPE_FIXED       0x00
#define RF_OOKPEAK_THRESHTYPE_PEAK        0x40  // Default
#define RF_OOKPEAK_THRESHTYPE_AVERAGE     0x80

#define RF_OOKPEAK_PEAKTHRESHSTEP_000     0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHSTEP_001     0x08
#define RF_OOKPEAK_PEAKTHRESHSTEP_010     0x10
#define RF_OOKPEAK_PEAKTHRESHSTEP_011     0x18
#define RF_OOKPEAK_PEAKTHRESHSTEP_100     0x20
#define RF_OOKPEAK_PEAKTHRESHSTEP_101     0x28
#define RF_OOKPEAK_PEAKTHRESHSTEP_110     0x30
#define RF_OOKPEAK_PEAKTHRESHSTEP_111     0x38

#define RF_OOKPEAK_PEAKTHRESHDEC_000      0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHDEC_001      0x01
#define RF_OOKPEAK_PEAKTHRESHDEC_010      0x02
#define RF_OOKPEAK_PEAKTHRESHDEC_011      0x03
#define RF_OOKPEAK_PEAKTHRESHDEC_100      0x04
#define RF_OOKPEAK_PEAKTHRESHDEC_101      0x05
#define RF_OOKPEAK_PEAKTHRESHDEC_110      0x06
#define RF_OOKPEAK_PEAKTHRESHDEC_111      0x07


// RegOokAvg
#define RF_OOKAVG_AVERAGETHRESHFILT_00    0x00
#define RF_OOKAVG_AVERAGETHRESHFILT_01    0x40
#define RF_OOKAVG_AVERAGETHRESHFILT_10    0x80  // Default
#define RF_OOKAVG_AVERAGETHRESHFILT_11    0xC0


// RegOokFix
#define RF_OOKFIX_FIXEDTHRESH_VALUE       0x06  // Default


// RegAfcFei
#define RF_AFCFEI_FEI_DONE                0x40
#define RF_AFCFEI_FEI_START               0x20
#define RF_AFCFEI_AFC_DONE                0x10
#define RF_AFCFEI_AFCAUTOCLEAR_ON         0x08
#define RF_AFCFEI_AFCAUTOCLEAR_OFF        0x00  // Default

#define RF_AFCFEI_AFCAUTO_ON              0x04
#define RF_AFCFEI_AFCAUTO_OFF             0x00  // Default

#define RF_AFCFEI_AFC_CLEAR               0x02
#define RF_AFCFEI_AFC_START               0x01


// RegRssiConfig
#define RF_RSSI_FASTRX_ON                 0x08  // not present on RFM69/SX1231
#define RF_RSSI_FASTRX_OFF                0x00  // Default

#define RF_RSSI_DONE                      0x02
#define RF_RSSI_START                     0x01


// RegDioMapping1
#define RF_DIOMAPPING1_DIO0_00            0x00  // Default
#define RF_DIOMAPPING1_DIO0_01            0x40
#define RF_DIOMAPPING1_DIO0_10            0x80
#define RF_DIOMAPPING1_DIO0_11            0xC0

#define RF_DIOMAPPING1_DIO1_00            0x00  // Default
#define RF_DIOMAPPING1_DIO1_01            0x10
#define RF_DIOMAPPING1_DIO1_10            0x20
#define RF_DIOMAPPING1_DIO1_11            0x30

#define RF_DIOMAPPING1_DIO2_00            0x00  // Default
#define RF_DIOMAPPING1_DIO2_01            0x04
#define RF_DIOMAPPING1_DIO2_10            0x08
#define RF_DIOMAPPING1_DIO2_11            0x0C

#define RF_DIOMAPPING1_DIO3_00            0x00  // Default
#define RF_DIOMAPPING1_DIO3_01            0x01
#define RF_DIOMAPPING1_DIO3_10            0x02
#define RF_DIOMAPPING1_DIO3_11            0x03


// RegDioMapping2
#define RF_DIOMAPPING2_DIO4_00            0x00  // Default
#define RF_DIOMAPPING2_DIO4_01            0x40
#define RF_DIOMAPPING2_DIO4_10            0x80
#define RF_DIOMAPPING2_DIO4_11            0xC0

#define RF_DIOMAPPING2_DIO5_00            0x00  // Default
#define RF_DIOMAPPING2_DIO5_01            0x10
#define RF_DIOMAPPING2_DIO5_10            0x20
#define RF_DIOMAPPING2_DIO5_11            0x30

#define RF_DIOMAPPING2_CLKOUT_32          0x00
#define RF_DIOMAPPING2_CLKOUT_16          0x01
#define RF_DIOMAPPING2_CLKOUT_8           0x02
#define RF_DIOMAPPING2_CLKOUT_4           0x03
#define RF_DIOMAPPING2_CLKOUT_2           0x04
#define RF_DIOMAPPING2_CLKOUT_1           0x05  // Reset value
#define RF_DIOMAPPING2_CLKOUT_RC          0x06
#define RF_DIOMAPPING2_CLKOUT_OFF         0x07  // Recommended default


// RegIrqFlags1
#define RF_IRQFLAGS1_MODEREADY            0x80
#define RF_IRQFLAGS1_RXREADY              0x40
#define RF_IRQFLAGS1_TXREADY              0x20
#define RF_IRQFLAGS1_PLLLOCK              0x10
#define RF_IRQFLAGS1_RSSI                 0x08
#define RF_IRQFLAGS1_TIMEOUT              0x04
#define RF_IRQFLAGS1_AUTOMODE             0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH     0x01


// RegIrqFlags2
#define RF_IRQFLAGS2_FIFOFULL             0x80
#define RF_IRQFLAGS2_FIFONOTEMPTY         0x40
#define RF_IRQFLAGS2_FIFOLEVEL            0x20
#define RF_IRQFLAGS2_FIFOOVERRUN          0x10
#define RF_IRQFLAGS2_PACKETSENT           0x08
#define RF_IRQFLAGS2_PAYLOADREADY         0x04
#define RF_IRQFLAGS2_CRCOK                0x02
#define RF_IRQFLAGS2_LOWBAT               0x01  // not present on RFM69/SX1231


// RegRssiThresh
#define RF_RSSITHRESH_VALUE               0xE4  // Default


// RegRxTimeout1
#define RF_RXTIMEOUT1_RXSTART_VALUE       0x00  // Default


// RegRxTimeout2
#define RF_RXTIMEOUT2_RSSITHRESH_VALUE    0x00  // Default


// RegPreamble
#define RF_PREAMBLESIZE_MSB_VALUE         0x00  // Default
#define RF_PREAMBLESIZE_LSB_VALUE         0x03  // Default


// RegSyncConfig
#define RF_SYNC_ON                0x80  // Default
#define RF_SYNC_OFF               0x00

#define RF_SYNC_FIFOFILL_AUTO     0x00  // Default -- when sync interrupt occurs
#define RF_SYNC_FIFOFILL_MANUAL   0x40

#define RF_SYNC_SIZE_1            0x00
#define RF_SYNC_SIZE_2            0x08
#define RF_SYNC_SIZE_3            0x10
#define RF_SYNC_SIZE_4            0x18  // Default
#define RF_SYNC_SIZE_5            0x20
#define RF_SYNC_SIZE_6            0x28
#define RF_SYNC_SIZE_7            0x30
#define RF_SYNC_SIZE_8            0x38

#define RF_SYNC_TOL_0             0x00  // Default
#define RF_SYNC_TOL_1             0x01
#define RF_SYNC_TOL_2             0x02
#define RF_SYNC_TOL_3             0x03
#define RF_SYNC_TOL_4             0x04
#define RF_SYNC_TOL_5             0x05
#define RF_SYNC_TOL_6             0x06
#define RF_SYNC_TOL_7             0x07


// RegSyncValue1-8
#define RF_SYNC_BYTE1_VALUE       0x00  // Default
#define RF_SYNC_BYTE2_VALUE       0x00  // Default
#define RF_SYNC_BYTE3_VALUE       0x00  // Default
#define RF_SYNC_BYTE4_VALUE       0x00  // Default
#define RF_SYNC_BYTE5_VALUE       0x00  // Default
#define RF_SYNC_BYTE6_VALUE       0x00  // Default
#define RF_SYNC_BYTE7_VALUE       0x00  // Default
#define RF_SYNC_BYTE8_VALUE       0x00  // Default


// RegPacketConfig1
#define RF_PACKET1_FORMAT_FIXED       0x00  // Default
#define RF_PACKET1_FORMAT_VARIABLE    0x80

#define RF_PACKET1_DCFREE_OFF         0x00  // Default
#define RF_PACKET1_DCFREE_MANCHESTER  0x20
#define RF_PACKET1_DCFREE_WHITENING   0x40

#define RF_PACKET1_CRC_ON             0x10  // Default
#define RF_PACKET1_CRC_OFF            0x00

#define RF_PACKET1_CRCAUTOCLEAR_ON    0x00  // Default
#define RF_PACKET1_CRCAUTOCLEAR_OFF   0x08

#define RF_PACKET1_ADRSFILTERING_OFF            0x00  // Default
#define RF_PACKET1_ADRSFILTERING_NODE           0x02
#define RF_PACKET1_ADRSFILTERING_NODEBROADCAST  0x04


// RegPayloadLength
#define RF_PAYLOADLENGTH_VALUE          0x40  // Default


// RegBroadcastAdrs
#define RF_BROADCASTADDRESS_VALUE       0x00


// RegAutoModes
#define RF_AUTOMODES_ENTER_OFF                0x00  // Default
#define RF_AUTOMODES_ENTER_FIFONOTEMPTY       0x20
#define RF_AUTOMODES_ENTER_FIFOLEVEL          0x40
#define RF_AUTOMODES_ENTER_CRCOK              0x60
#define RF_AUTOMODES_ENTER_PAYLOADREADY       0x80
#define RF_AUTOMODES_ENTER_SYNCADRSMATCH      0xA0
#define RF_AUTOMODES_ENTER_PACKETSENT         0xC0
#define RF_AUTOMODES_ENTER_FIFOEMPTY          0xE0

#define RF_AUTOMODES_EXIT_OFF                 0x00  // Default
#define RF_AUTOMODES_EXIT_FIFOEMPTY           0x04
#define RF_AUTOMODES_EXIT_FIFOLEVEL           0x08
#define RF_AUTOMODES_EXIT_CRCOK               0x0C
#define RF_AUTOMODES_EXIT_PAYLOADREADY        0x10
#define RF_AUTOMODES_EXIT_SYNCADRSMATCH       0x14
#define RF_AUTOMODES_EXIT_PACKETSENT          0x18
#define RF_AUTOMODES_EXIT_RXTIMEOUT           0x1C

#define RF_AUTOMODES_INTERMEDIATE_SLEEP       0x00  // Default
#define RF_AUTOMODES_INTERMEDIATE_STANDBY     0x01
#define RF_AUTOMODES_INTERMEDIATE_RECEIVER    0x02
#define RF_AUTOMODES_INTERMEDIATE_TRANSMITTER 0x03


// RegFifoThresh
#define RF_FIFOTHRESH_TXSTART_FIFOTHRESH      0x00  // Reset value
#define RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY    0x80  // Recommended default

#define RF_FIFOTHRESH_VALUE                   0x0F  // Default


// RegPacketConfig2
#define RF_PACKET2_RXRESTARTDELAY_1BIT        0x00  // Default
#define RF_PACKET2_RXRESTARTDELAY_2BITS       0x10
#define RF_PACKET2_RXRESTARTDELAY_4BITS       0x20
#define RF_PACKET2_RXRESTARTDELAY_8BITS       0x30
#define RF_PACKET2_RXRESTARTDELAY_16BITS      0x40
#define RF_PACKET2_RXRESTARTDELAY_32BITS      0x50
#define RF_PACKET2_RXRESTARTDELAY_64BITS      0x60
#define RF_PACKET2_RXRESTARTDELAY_128BITS     0x70
#define RF_PACKET2_RXRESTARTDELAY_256BITS     0x80
#define RF_PACKET2_RXRESTARTDELAY_512BITS     0x90
#define RF_PACKET2_RXRESTARTDELAY_1024BITS    0xA0
#define RF_PACKET2_RXRESTARTDELAY_2048BITS    0xB0
#define RF_PACKET2_RXRESTARTDELAY_NONE        0xC0
#define RF_PACKET2_RXRESTART                  0x04

#define RF_PACKET2_AUTORXRESTART_ON           0x02  // Default
#define RF_PACKET2_AUTORXRESTART_OFF          0x00

#define RF_PACKET2_AES_ON                     0x01
#define RF_PACKET2_AES_OFF                    0x00  // Default


// RegAesKey1-16
#define RF_AESKEY1_VALUE            0x00  // Default
#define RF_AESKEY2_VALUE            0x00  // Default
#define RF_AESKEY3_VALUE            0x00  // Default
#define RF_AESKEY4_VALUE            0x00  // Default
#define RF_AESKEY5_VALUE            0x00  // Default
#define RF_AESKEY6_VALUE            0x00  // Default
#define RF_AESKEY7_VALUE            0x00  // Default
#define RF_AESKEY8_VALUE            0x00  // Default
#define RF_AESKEY9_VALUE            0x00  // Default
#define RF_AESKEY10_VALUE           0x00  // Default
#define RF_AESKEY11_VALUE           0x00  // Default
#define RF_AESKEY12_VALUE           0x00  // Default
#define RF_AESKEY13_VALUE           0x00  // Default
#define RF_AESKEY14_VALUE           0x00  // Default
#define RF_AESKEY15_VALUE           0x00  // Default
#define RF_AESKEY16_VALUE           0x00  // Default


// RegTemp1
#define RF_TEMP1_MEAS_START         0x08
#define RF_TEMP1_MEAS_RUNNING       0x04
// not present on RFM69/SX1231
#define RF_TEMP1_ADCLOWPOWER_ON     0x01  // Default
#define RF_TEMP1_ADCLOWPOWER_OFF    0x00


// RegTestLna
#define RF_TESTLNA_NORMAL           0x1B
#define RF_TESTLNA_HIGH_SENSITIVITY 0x2D


// RegTestDagc
#define RF_DAGC_NORMAL              0x00  // Reset value
#define RF_DAGC_IMPROVED_LOWBETA1   0x20
#define RF_DAGC_IMPROVED_LOWBETA0   0x30  // Recommended default

#endif

#ifdef OUR_REG_DEFINE

// RFM69W Register definitions   Adress:  Reset:    Recomended: Description:
#define RFM69W_RegFifo           0x00  // Def: 0x00,            FIFO read/write access 
#define RFM69W_RegOpMode         0x01  // Def: 0x04,            Operating modes of the transceiver 
#define RFM69W_RegDataModul      0x02  // Def: 0x00,            Data operation mode and Modulation settings 
#define RFM69W_RegBitrateMsb     0x03  // Def: 0x1A,            Bit Rate setting, Most Significant Bits 
#define RFM69W_RegBitrateLsb     0x04  // Def: 0x0B,            Bit Rate setting, Least Significant Bits 
#define RFM69W_RegFdevMsb        0x05  // Def: 0x00,            Frequency Deviation setting, Most Significant Bits 
#define RFM69W_RegFdevLsb        0x06  // Def: 0x52,            Frequency Deviation setting, Least Significant Bits 
#define RFM69W_RegFrfMsb         0x07  // Def: 0xE4,            RF Carrier Frequency, Most Significant Bits 
#define RFM69W_RegFrfMid         0x08  // Def: 0xC0,            RF Carrier Frequency, Intermediate Bits 
#define RFM69W_RegFrfLsb         0x09  // Def: 0x00,            RF Carrier Frequency, Least Significant Bits 
#define RFM69W_RegOsc1           0x0A  // Def: 0x41,            RC Oscillators Settings 
#define RFM69W_RegAfcCtrl        0x0B  // Def: 0x00,            AFC control in low modulation index situations 
#define RFM69W_Reserved0C        0x0C  // Def: 0x02, - 
#define RFM69W_RegListen1        0x0D  // Def: 0x92,            Listen Mode settings 
#define RFM69W_RegListen2        0x0E  // Def: 0xF5,            Listen Mode Idle duration 
#define RFM69W_RegListen3        0x0F  // Def: 0x20,            Listen Mode Rx duration 
#define RFM69W_RegVersion        0x10  // Def: 0x24, 
#define RFM69W_RegPaLevel        0x11  // Def: 0x9F,            PA selection and Output Power control 
#define RFM69W_RegPaRamp         0x12  // Def: 0x09,            Control of the PA ramp time in FSK mode 
#define RFM69W_RegOcp            0x13  // Def: 0x1A,            Over Current Protection control 
#define RFM69W_Reserved14        0x14  // Def: 0x40, -
#define RFM69W_Reserved15        0x15  // Def: 0xB0, -
#define RFM69W_Reserved16        0x16  // Def: 0x7B, -
#define RFM69W_Reserved17        0x17  // Def: 0x9B, -
#define RFM69W_RegLna            0x18  // Def: 0x08, Rec: 0x88, LNA settings 
#define RFM69W_RegRxBw           0x19  // Def: 0x86, Rec: 0x55, Channel Filter BW Control 
#define RFM69W_RegAfcBw          0x1A  // Def: 0x8A, Rec: 0x8B, Channel Filter BW control during the AFC routine 
#define RFM69W_RegOokPeak        0x1B  // Def: 0x40,            OOK demodulator selection and control in peak mode 
#define RFM69W_RegOokAvg         0x1C  // Def: 0x80,            Average threshold control of the OOK demodulator 
#define RFM69W_RegOokFix         0x1D  // Def: 0x06,            Fixed threshold control of the OOK demodulator 
#define RFM69W_RegAfcFei         0x1E  // Def: 0x10,            AFC and FEI control and status 
#define RFM69W_RegAfcMsb         0x1F  // Def: 0x00,            MSB of the frequency correction of the AFC 
#define RFM69W_RegAfcLsb         0x20  // Def: 0x00,            LSB of the frequency correction of the AFC 
#define RFM69W_RegFeiMsb         0x21  // Def: 0x00,            MSB of the calculated frequency error 
#define RFM69W_RegFeiLsb         0x22  // Def: 0x00,            LSB of the calculated frequency error 
#define RFM69W_RegRssiConfig     0x23  // Def: 0x02,            RSSI-related settings 
#define RFM69W_RegRssiValue      0x24  // Def: 0xFF,            RSSI value in dBm 
#define RFM69W_RegDioMapping1    0x25  // Def: 0x00,            Mapping of pins DIO0 to DIO3 
#define RFM69W_RegDioMapping2    0x26  // Def: 0x05, Rec: 0x07, Mapping of pins DIO4 and DIO5, ClkOut frequency 
#define RFM69W_RegIrqFlags1      0x27  // Def: 0x80,            Status register: PLL Lock state, Timeout, RSSI > Threshold... 
#define RFM69W_RegIrqFlags2      0x28  // Def: 0x00,            Status register: FIFO handling flags... 
#define RFM69W_RegRssiThresh     0x29  // Def: 0xFF, Rec: 0xE4, RSSI Threshold control 
#define RFM69W_RegRxTimeout1     0x2A  // Def: 0x00,            Timeout duration between Rx request and RSSI detection 
#define RFM69W_RegRxTimeout2     0x2B  // Def: 0x00,            Timeout duration between RSSI detection and PayloadReady  
#define RFM69W_RegPreambleMsb    0x2C  // Def: 0x00,            Preamble length, MSB 
#define RFM69W_RegPreambleLsb    0x2D  // Def: 0x03,            Preamble length, LSB 
#define RFM69W_RegSyncConfig     0x2E  // Def: 0x98,            Sync Word Recognition control 
#define RFM69W_RegSyncValue1     0x2F  // Def: 0x00, Rec: 0x01, Sync Word byte 1
#define RFM69W_RegSyncValue2     0x30  // Def: 0x00, Rec: 0x01, Sync Word byte 2
#define RFM69W_RegSyncValue3     0x31  // Def: 0x00, Rec: 0x01, Sync Word byte 3 
#define RFM69W_RegSyncValue4     0x32  // Def: 0x00, Rec: 0x01, Sync Word byte 4 
#define RFM69W_RegSyncValue5     0x33  // Def: 0x00, Rec: 0x01, Sync Word byte 5 
#define RFM69W_RegSyncValue6     0x34  // Def: 0x00, Rec: 0x01, Sync Word byte 6 
#define RFM69W_RegSyncValue7     0x35  // Def: 0x00, Rec: 0x01, Sync Word byte 7 
#define RFM69W_RegSyncValue8     0x36  // Def: 0x00, Rec: 0x01, Sync Word byte 8 
#define RFM69W_RegPacketConfig1  0x37  // Def: 0x10,            Packet mode settings 
#define RFM69W_RegPayloadLength  0x38  // Def: 0x40,            Payload length setting 
#define RFM69W_RegNodeAdrs       0x39  // Def: 0x00,            Node address 
#define RFM69W_RegBroadcastAdrs  0x3A  // Def: 0x00,            Broadcast address 
#define RFM69W_RegAutoModes      0x3B  // Def: 0x00,            Auto modes settings 
#define RFM69W_RegFifoThresh     0x3C  // Def: 0x0F, Rec: 0x8F, Fifo threshold, Tx start condition 
#define RFM69W_RegPacketConfig2  0x3D  // Def: 0x02,            Packet mode settings 
#define RFM69W_RegAesKey1        0x3E  // Def: 0x00,            1 byte of the cypher key 
#define RFM69W_RegAesKey2        0x3F  // Def: 0x00,            2 byte of the cypher key
#define RFM69W_RegAesKey3        0x40  // Def: 0x00,            3 byte of the cypher key
#define RFM69W_RegAesKey4        0x41  // Def: 0x00,            4 byte of the cypher key
#define RFM69W_RegAesKey5        0x42  // Def: 0x00,            5 byte of the cypher key
#define RFM69W_RegAesKey6        0x43  // Def: 0x00,            6 byte of the cypher key
#define RFM69W_RegAesKey7        0x44  // Def: 0x00,            7 byte of the cypher key
#define RFM69W_RegAesKey8        0x45  // Def: 0x00,            8 byte of the cypher key
#define RFM69W_RegAesKey9        0x46  // Def: 0x00,            9 byte of the cypher key
#define RFM69W_RegAesKey10       0x47  // Def: 0x00,            10 byte of the cypher key
#define RFM69W_RegAesKey11       0x48  // Def: 0x00,            11 byte of the cypher key
#define RFM69W_RegAesKey12       0x49  // Def: 0x00,            12 byte of the cypher key
#define RFM69W_RegAesKey13       0x4A  // Def: 0x00,            13 byte of the cypher key
#define RFM69W_RegAesKey14       0x4B  // Def: 0x00,            14 byte of the cypher key
#define RFM69W_RegAesKey15       0x4C  // Def: 0x00,            15 byte of the cypher key
#define RFM69W_RegAesKey16       0x4D  // Def: 0x00,            16 byte of the cypher key
#define RFM69W_RegTemp1          0x4E  // Def: 0x01,            Temperature Sensor control 
#define RFM69W_RegTemp2          0x4F  // Def: 0x00,            Temperature readout 
#define RFM69W_RegTestLna        0x58  // Def: 0x1B,            Sensitivity boost 
#define RFM69W_RegTestPa1        0x5A  // Def: 0x55,            High Power PA settings 
#define RFM69W_RegTestPa2        0x5C  // Def: 0x70,            High Power PA settings 
#define RFM69W_RegTestDagc       0x6F  // Def: 0x00, Rec: 0x30, Fading Margin Improvement 
#define RFM69W_RegTestAfc        0x71  // Def: 0x00,            AFC offset for low modulation index AFC  

// RegOpMode

#define RegOpMode_SequencerOff                       0x80
#define RegOpMode_ListenOn                           0x70
#define RegOpMode_ListenAbort                        0x60
#define RegOpMode_Mode_SLEEP                         0x00
#define RegOpMode_Mode_STDBY                         0x04
#define RegOpMode_Mode_FS                            0x08
#define RegOpMode_Mode_TX                            0x0C
#define RegOpMode_Mode_RX                            0x10

// RegDataModul

#define RegDataModul_DataMode_packet                 0x00
#define RegDataModul_DataMode_cont_sync              0x40
#define RegDataModul_DataMode_cont_nosync            0x60
#define RegDataModul_ModulationType_FSK              0x00
#define RegDataModul_ModulationType_OOK              0x08
#define RegDataModul_ModulationShaping_noshape       0x00
#define RegDataModul_ModulationShaping_FSK_gauss_1   0x01
#define RegDataModul_ModulationShaping_FSK_gauss_0_5 0x02
#define RegDataModul_ModulationShaping_FSK_gauss_0_3 0x03
#define RegDataModul_ModulationShaping_OOK_BR        0x01
#define RegDataModul_ModulationShaping_OOK_2BR       0x02

// RegFdevMsb

#define RegFdevMsb_freq_mask                         0x3F

// RegOsc1

#define RegOsc1_RcCalStart                           0x80
#define RegOsc1_RcCalDone                            0x40

// RegAfcCtrl

#define RegAfcCtrl_AfcLowBetaOn                      0x20

// RegListen1

#define RegListen1_ListenResolIdle_64                0x40
#define RegListen1_ListenResolIdle_4_1               0x80
#define RegListen1_ListenResolIdle_262               0xC0
#define RegListen1_ListenResolRx_64                  0x10
#define RegListen1_ListenResolRx_4_1                 0x20
#define RegListen1_ListenResolRx_262                 0x30
#define RegListen1_ListenCriteria_1                  0x08
#define RegListen1_ListenCriteria_0                  0x00
#define RegListen1_ListenEnd_Rx_0                    0x00
#define RegListen1_ListenEnd_Rx_1                    0x01
#define RegListen1_ListenEnd_Rx_2                    0x02

// RegPaLevel

#define RegPaLevel_Pa0On                             0x80
#define RegPaLevel_Pa1On                             0x40
#define RegPaLevel_Pa2On                             0x20
#define RegPaLevel_OutputPower_mask                  0x1F

// RegPaRamp

#define RegPaRamp_PaRamp_3_4ms                       0x00
#define RegPaRamp_PaRamp_2ms                         0x01
#define RegPaRamp_PaRamp_1ms                         0x02
#define RegPaRamp_PaRamp_500us                       0x03
#define RegPaRamp_PaRamp_250us                       0x04
#define RegPaRamp_PaRamp_125us                       0x05
#define RegPaRamp_PaRamp_100us                       0x06
#define RegPaRamp_PaRamp_62us                        0x07
#define RegPaRamp_PaRamp_50us                        0x08
#define RegPaRamp_PaRamp_40us                        0x09
#define RegPaRamp_PaRamp_31us                        0x0A
#define RegPaRamp_PaRamp_25us                        0x0B
#define RegPaRamp_PaRamp_20us                        0x0C
#define RegPaRamp_PaRamp_15us                        0x0D
#define RegPaRamp_PaRamp_12us                        0x0E
#define RegPaRamp_PaRamp_10us                        0x0F

// RegOcp

#define RegOcp_OcpOn_1                               0x10
#define RegOcp_OcpOn_0                               0x00
#define RegOcp_OcpTrim_mask                          0x0F

// RegLna

#define RegLna_LnaZin_50Ohm                          0x00
#define RegLna_LnaZin_200Ohm                         0x80
#define RegLna_LnaCurrentGain_mask                   0x38
#define RegLna_LnaGainSelect_AGC                     0x00
#define RegLna_LnaGainSelect_high                    0x01
#define RegLna_LnaGainSelect_6dB                     0x02
#define RegLna_LnaGainSelect_12dB                    0x03
#define RegLna_LnaGainSelect_24dB                    0x04
#define RegLna_LnaGainSelect_36dB                    0x05
#define RegLna_LnaGainSelect_48dB                    0x06

// RegRxBw

#define RegRxBw_DccFreq_mask                         0xE0
#define RegRxBw_RxBwMant_16                          0x00
#define RegRxBw_RxBwMant_20                          0x08
#define RegRxBw_RxBwMant_24                          0x10
#define RegRxBw_RxBwExp_mask                         0x07

// RegAfcBw

#define RegAfcBw_DccFreqAfc_mask                     0xE0
#define RegAfcBw_RxBwMantAfc_mask                    0x18
#define RegAfcBw_RxBwExpAfc_mask                     0x07

// RegOokPeak

#define RegOokPeak_OokThreshType_fixed               0x00
#define RegOokPeak_OokThreshType_peak                0x40
#define RegOokPeak_OokThreshType_average             0x80
#define RegOokPeak_OokPeakTheshStep_0_5dB            0x00
#define RegOokPeak_OokPeakTheshStep_1_0dB            0x08
#define RegOokPeak_OokPeakTheshStep_1_5dB            0x10
#define RegOokPeak_OokPeakTheshStep_2_0dB            0x18
#define RegOokPeak_OokPeakTheshStep_3_0dB            0x20
#define RegOokPeak_OokPeakTheshStep_4_0dB            0x28
#define RegOokPeak_OokPeakTheshStep_5_0dB            0x30
#define RegOokPeak_OokPeakTheshStep_6_0dB            0x38
#define RegOokPeak_OokPeakThreshDec_1_1chip          0x00
#define RegOokPeak_OokPeakThreshDec_1_2chip          0x01
#define RegOokPeak_OokPeakThreshDec_1_4chip          0x02
#define RegOokPeak_OokPeakThreshDec_1_8chip          0x03
#define RegOokPeak_OokPeakThreshDec_2_Xchip          0x04
#define RegOokPeak_OokPeakThreshDec_4_Xchip          0x05
#define RegOokPeak_OokPeakThreshDec_8_Xchip          0x06
#define RegOokPeak_OokPeakThreshDec_16_Xchip         0x07

// RegOokAvg

#define RegOokAvg_OokAverageThreshFilt_chip_32       0x00
#define RegOokAvg_OokAverageThreshFilt_chip_8        0x40
#define RegOokAvg_OokAverageThreshFilt_chip_4        0x80
#define RegOokAvg_OokAverageThreshFilt_chip_2        0xC0

// RegAfcFei

#define RegAfcFei_FeiDone                            0x40
#define RegAfcFei_FeiStart                           0x20
#define RegAfcFei_AfcDone                            0x10
#define RegAfcFei_AfcAutoclearOn_1                   0x08
#define RegAfcFei_AfcAutoclearOn_0                   0x00
#define RegAfcFei_AfcAutoOn_1                        0x04
#define RegAfcFei_AfcAutoOn_0                        0x00
#define RegAfcFei_AfcClear                           0x02
#define RegAfcFei_AfcStart                           0x01

// RegRssiConfig

#define RegRssiConfig_RssiDone                       0x02
#define RegRssiConfig_RssiStart                      0x01

// RegDioMapping1

#define RegDioMapping1_Dio0Mapping_mask              0xC0
#define RegDioMapping1_Dio1Mapping_mask              0x30
#define RegDioMapping1_Dio2Mapping_mask              0x0C
#define RegDioMapping1_Dio3Mapping_mask              0x03

// RegDioMapping2

#define RegDioMapping2_Dio4Mapping_mask              0xC0
#define RegDioMapping2_Dio5Mapping_mask              0x30
#define RegDioMapping2_ClkOut_FXOSC                  0x00
#define RegDioMapping2_ClkOut_FXOSC_2                0x01
#define RegDioMapping2_ClkOut_FXOSC_4                0x02
#define RegDioMapping2_ClkOut_FXOSC_8                0x03
#define RegDioMapping2_ClkOut_FXOSC_16               0x04
#define RegDioMapping2_ClkOut_FXOSC_32               0x05
#define RegDioMapping2_ClkOut_FXOSC_RC               0x06
#define RegDioMapping2_ClkOut_FXOSC_OFF              0x07

// RegIrqFlags1

#define RegIrqFlags1_ModeReady                       0x80
#define RegIrqFlags1_RxReady                         0x40
#define RegIrqFlags1_TxReady                         0x20
#define RegIrqFlags1_PllLock                         0x10
#define RegIrqFlags1_Rssi                            0x08
#define RegIrqFlags1_Timeout                         0x04
#define RegIrqFlags1_AutoMode                        0x02
#define RegIrqFlags1_SyncAddressMatch                0x01

// RegIrqFlags2

#define RegIrqFlags2_FifoFull                        0x80
#define RegIrqFlags2_FifoNotEmpty                    0x40
#define RegIrqFlags2_FifoLevel                       0x20
#define RegIrqFlags2_FifoOverrun                     0x10
#define RegIrqFlags2_PacketSent                      0x08
#define RegIrqFlags2_PayloadReady                    0x04
#define RegIrqFlags2_CrcOk                           0x02

// RegSyncConfig

#define RegSyncConfig_SyncOn_1                       0x80
#define RegSyncConfig_SyncOn_0                       0x00
#define RegSyncConfig_FifoFillCondition_1            0x40
#define RegSyncConfig_FifoFillCondition_0            0x00
#define RegSyncConfig_SyncSize_mask                  0x38
#define RegSyncConfig_SyncTol_mask                   0x07

// RegPacketConfig1

#define RegPacketConfig1_PacketFormat_1              0x80
#define RegPacketConfig1_PacketFormat_0              0x00
#define RegPacketConfig1_DcFree_None                 0x00
#define RegPacketConfig1_DcFree_Manchester           0x20
#define RegPacketConfig1_DcFree_Whitening            0x40
#define RegPacketConfig1_CrcOn_1                     0x10
#define RegPacketConfig1_CrcOn_0                     0x00
#define RegPacketConfig1_CrcAutoClearOff_1           0x08
#define RegPacketConfig1_CrcAutoClearOff_0           0x00
#define RegPacketConfig1_AddressFiltering_None       0x00
#define RegPacketConfig1_AddressFiltering_Node       0x01
#define RegPacketConfig1_AddressFiltering_Node_Broad 0x02

// RegAutoModes

#define RegAutoModes_EnterCondition_None             0x00
#define RegAutoModes_EnterCondition_R_FifoNotEmpty   0x20
#define RegAutoModes_EnterCondition_R_FifoLevel      0x40
#define RegAutoModes_EnterCondition_R_CrcOk          0x60
#define RegAutoModes_EnterCondition_R_PayloadReady   0x80
#define RegAutoModes_EnterCondition_R_SyncAddress    0xA0
#define RegAutoModes_EnterCondition_R_PacketSent     0xC0
#define RegAutoModes_EnterCondition_F_FifoNotEmpty   0xE0
#define RegAutoModes_ExitCondition_None              0x00
#define RegAutoModes_ExitCondition_F_FifoNotEmpty    0x04
#define RegAutoModes_ExitCondition_R_FifoLevel       0x08
#define RegAutoModes_ExitCondition_R_CrcOk           0x0C
#define RegAutoModes_ExitCondition_R_PayloadReady    0x10
#define RegAutoModes_ExitCondition_R_SyncAddress     0x14
#define RegAutoModes_ExitCondition_R_PacketSent      0x18
#define RegAutoModes_ExitCondition_R_Timeout         0x1C
#define RegAutoModes_IntermediateMode_SLEEP          0x00
#define RegAutoModes_IntermediateMode_STDBY          0x01
#define RegAutoModes_IntermediateMode_RX             0x02
#define RegAutoModes_IntermediateMode_TX             0x03

// RegFifoThresh

#define RegFifoThresh_TxStartCondition_1             0x80
#define RegFifoThresh_TxStartCondition_0             0x00
#define RegFifoThresh_FifoThreshold_mask             0x7F

// RegPacketConfig2

#define RegPacketConfig2_InterPacketRxDelay_mask     0xF0
#define RegPacketConfig2_RestartRx                   0x04
#define RegPacketConfig2_AutoRxRestartOn_1           0x02
#define RegPacketConfig2_AutoRxRestartOn_0           0x00
#define RegPacketConfig2_AesOn_1                     0x01
#define RegPacketConfig2_AesOn_0                     0x00

// RegTemp1

#define RegTemp1_TempMeasStart                       0x08
#define RegTemp1_TempMeasRunning                     0x04

#endif

#endif
