#ifndef __EEPROM_H
#define __EEPROM_H

// Public functions
void _EEPROM_64_erase(unsigned int ADDRESS);
void _EEPROM_32_erase(unsigned int ADDRESS);

void _EEPROM_64_write(unsigned int ADDRESS, uint64_t DATA);
void _EEPROM_32_write(unsigned int ADDRESS, unsigned int DATA);
void _EEPROM_16_write(unsigned int ADDRESS, unsigned short int DATA);
void _EEPROM_8_write(unsigned int ADDRESS, unsigned char DATA);

void _EEPROM_32_fast_write(unsigned int ADDRESS, unsigned int DATA);
void _EEPROM_16_fast_write(unsigned int ADDRESS, unsigned short int DATA);
void _EEPROM_8_fast_write(unsigned int ADDRESS, unsigned char DATA);

uint64_t _EEPROM_64_read(unsigned int ADDRESS);
unsigned int _EEPROM_32_read(unsigned int ADDRESS);
unsigned short int _EEPROM_16_read(unsigned int ADDRESS);
unsigned char _EEPROM_8_read(unsigned int ADDRESS);
// End of public functions

#endif
