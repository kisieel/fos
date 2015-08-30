#include "stm32l1xx.h"

#include "RFM69W.h"
#include "USART.h"
#include "EEPROM.h"
#include "KEY.h"
#include "MENU.h"
#include "HALL.h"
#include "SYS_TICK.h"

#include "FLOAT.h"

// Comment following line if you don't want to receive USART messages
#define USART_debug             1

// Definitions for geting device electronic sgnature
// Usage: A = MMIO32(U_ID); B = MMIO32(U_ID + 0x04); C = MMIO32(U_ID + 0x14)
#define MMIO32(addr)            (*(volatile uint32_t *)(addr))
#define U_ID                    0x1FF80050

