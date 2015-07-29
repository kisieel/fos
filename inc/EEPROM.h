#ifndef __EEPROM_H
#define __EEPROM_H

// Public functions
extern void     EEPROM_64_erase(uint32_t ADDRESS);
extern void     EEPROM_32_erase(uint32_t ADDRESS);

extern void     EEPROM_64_write(uint32_t ADDRESS, uint64_t DATA);
extern void     EEPROM_32_write(uint32_t ADDRESS, uint32_t DATA);
extern void     EEPROM_16_write(uint32_t ADDRESS, uint16_t DATA);
extern void     EEPROM_8_write(uint32_t ADDRESS, uint8_t DATA);

extern void     EEPROM_32_fast_write(uint32_t ADDRESS, uint32_t DATA);
extern void     EEPROM_16_fast_write(uint32_t ADDRESS, uint16_t DATA);
extern void     EEPROM_8_fast_write(uint32_t ADDRESS, uint8_t DATA);

extern uint64_t EEPROM_64_read(uint32_t ADDRESS);
extern uint32_t EEPROM_32_read(uint32_t ADDRESS);
extern uint16_t EEPROM_16_read(uint32_t ADDRESS);
extern uint8_t  EEPROM_8_read(uint32_t ADDRESS);
// End of public functions

#endif
