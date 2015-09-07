#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l1xx.h"

#include "RFM69W.h"
#include "USART.h"
#include "EEPROM.h"
#include "KEY.h"
#include "MENU.h"
#include "HALL.h"
#include "SYS_TICK.h"
#include "BUZZER.h"
#include "LED.h"

#include "FLOAT.h"

// Definitions for geting device electronic signature
// Usage: A = MMIO32(U_ID); B = MMIO32(U_ID + 0x04); C = MMIO32(U_ID + 0x14)
#define MMIO32(addr)            (*(volatile uint32_t *)(addr))
#define U_ID                    0x1FF80050

#define TRUE      1
#define FALSE     0

// Comment following line if you don't want to receive USART messages
#define USART_debug             TRUE

// Time to hold the key to turn off
#define TurnOffTime             3000

#define PowerOn                 GPIOA->BSRRL |= GPIO_BSRR_BS_0;
#define PowerOff                GPIOA->BSRRH |= GPIO_BSRR_BR_0;

typedef struct {
	uint8_t ActAnimation;
	uint8_t ActColor;
	uint8_t ActBrightness;
	uint8_t ActAlarmTone;
	uint8_t ActAlarmVol;
	uint8_t ActAlarmTempo;
	uint8_t ActMusic;
} SystemType;

extern SystemType System;

#endif