#include "stm32l1xx.h"
#include "EEPROM.h"

/* 

Based on PM0062.
** DBL WORD-  64 bit
** WORD-      32 bit
** HALF WORD- 16 bit
** BYTE-      8 bit 

** Addresses are expressed in bytes

FAST WRITE- assuming value is already equal to zero. It takes
the 1 tprog to program the memory. If the value isn't equal to 
zero Flash interface clears it and then writes to it. It takes
2 tprog to program the memory. FTDW bit in PECR is responsible
for fast/normal operations.

*/

void EEPROM_unlock(void)
{
	FLASH->PEKEYR = (unsigned int)0x89ABCDEF;
	FLASH->PEKEYR = (unsigned int)0x02030405;
	
	while(FLASH->PECR & FLASH_PECR_PELOCK);
}

void EEPROM_lock(void)
{
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

void EEPROM_option_unlock(void)
{
	EEPROM_unlock();
	FLASH->OPTKEYR = (unsigned int)0xFBEAD9C8;
	FLASH->OPTKEYR = (unsigned int)0x24252627;
}

void EEPROM_option_lock(void)
{
	FLASH->PECR |= FLASH_PECR_OPTLOCK;
}

void EEPROM_32_erase(unsigned int ADDRESS)
{
	// Page 18
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	
	*ptr = (unsigned int)0x00000000;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_64_erase(unsigned int ADDRESS)
{
	// Page 19
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR |= FLASH_PECR_ERASE;
	FLASH->PECR |= FLASH_PECR_DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	
	*ptr = (unsigned int)0x00000000;
	ptr += 4;
	while(FLASH->SR & FLASH_SR_BSY);
	*ptr = (unsigned int)0x00000000;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_64_write(unsigned int ADDRESS, unsigned long int DATA)
{
	// Page 21
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR |= FLASH_PECR_FPRG;
	FLASH->PECR |= FLASH_PECR_DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	
	*ptr = (unsigned int)(DATA & 0x00000000FFFFFFFF);
	ptr += 4;
	while(FLASH->SR & FLASH_SR_BSY);
	*ptr = (unsigned int)((DATA & 0xFFFFFFFF00000000) >> 32);
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_32_fast_write(unsigned int ADDRESS, unsigned int DATA)
{
	// Page 22
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR &= ~(FLASH_PECR_FTDW);
	
	*ptr = (unsigned int)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_32_write(unsigned int ADDRESS, unsigned int DATA)
{
	// Page 22
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR |= FLASH_PECR_FTDW;
	
	*ptr = (unsigned int)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_16_fast_write(unsigned int ADDRESS, unsigned short int DATA)
{
	// Page 23
	unsigned short int *ptr;
	ptr = (unsigned short int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR &= ~(FLASH_PECR_FTDW);
	
	*ptr = (unsigned short int)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_16_write(unsigned int ADDRESS, unsigned short int DATA)
{
	// Page 23
	unsigned short int *ptr;
	ptr = (unsigned short int *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR |= FLASH_PECR_FTDW;
	
	*ptr = (unsigned short int)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_8_fast_write(unsigned int ADDRESS, unsigned char DATA)
{
	// Page 24
	unsigned char *ptr;
	ptr = (unsigned char *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR &= ~(FLASH_PECR_FTDW);
	
	*ptr = (unsigned char)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

void EEPROM_8_write(unsigned int ADDRESS, unsigned char DATA)
{
	// Page 24
	unsigned char *ptr;
	ptr = (unsigned char *)(EEPROM_BASE + ADDRESS);
	
	__disable_irq();
	EEPROM_unlock();
	FLASH->PECR |= FLASH_PECR_FTDW;
	
	*ptr = (unsigned char)DATA;
	while(FLASH->SR & FLASH_SR_BSY);
	FLASH->PECR |= FLASH_PECR_PELOCK;
	__enable_irq();
}

unsigned long int EEPROM_64_read(unsigned int ADDRESS)
{
	unsigned long int *ptr;
	ptr = (unsigned long int *)(EEPROM_BASE + ADDRESS);
	
	return *ptr;
}

unsigned int EEPROM_32_read(unsigned int ADDRESS)
{
	unsigned int *ptr;
	ptr = (unsigned int *)(EEPROM_BASE + ADDRESS);
	
	return *ptr;
}

unsigned short int EEPROM_16_read(unsigned int ADDRESS)
{
	unsigned short int *ptr;
	ptr = (unsigned short int *)(EEPROM_BASE + ADDRESS);
	
	return *ptr;
}

unsigned char EEPROM_8_read(unsigned int ADDRESS)
{
	unsigned char *ptr;
	ptr = (unsigned char *)(EEPROM_BASE + ADDRESS);
	
	return *ptr;
}