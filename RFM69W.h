#ifndef __RFM69W_H
#define __RFM69W_H

typedef struct {
	uint8_t Address;
	uint8_t Data;
	uint8_t Type;
	uint8_t Flag_address;
	uint8_t Flag_data;
} RFM69W_Queue;

#define RFM69W_Write    1
#define RFM69W_Read     0

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