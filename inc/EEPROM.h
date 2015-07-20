#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_BASE           ((uint32_t)0x08080000) /*!< EEPROM base address in the alias region */

void EEPROM_unlock(void);
void EEPROM_lock(void);
void EEPROM_option_unlock(void);
void EEPROM_option_lock(void);
void EEPROM_32_erase(unsigned int);
void EEPROM_64_erase(unsigned int);
void EEPROM_64_write(unsigned int, unsigned long int);
void EEPROM_32_fast_write(unsigned int, unsigned int);
void EEPROM_32_write(unsigned int, unsigned int);
void EEPROM_16_fast_write(unsigned int, unsigned short int);
void EEPROM_16_write(unsigned int, unsigned short int);
void EEPROM_8_fast_write(unsigned int, unsigned char);
void EEPROM_8_write(unsigned int, unsigned char);

unsigned long int EEPROM_64_read(unsigned int);
unsigned int EEPROM_32_read(unsigned int);
unsigned short int EEPROM_16_read(unsigned int);
unsigned char EEPROM_8_read(unsigned int);

#endif
