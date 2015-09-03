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

// Comment following line if you don't want to receive USART messages
//#define USART_debug             1

// Definitions for geting device electronic signature
// Usage: A = MMIO32(U_ID); B = MMIO32(U_ID + 0x04); C = MMIO32(U_ID + 0x14)
#define MMIO32(addr)            (*(volatile uint32_t *)(addr))
#define U_ID                    0x1FF80050

typedef struct {
	uint8_t ActAnimation;
	uint8_t ActTone;
	uint8_t ActColor;
} SystemType;

extern SystemType System;