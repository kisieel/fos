#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_ConfAddress1          0
#define EEPROM_ConfAddress2          4
#define EEPROM_ConfAddress3          8

#define EEPROM_1_ActAnimation          0x0000000F
#define EEPROM_1_ActAnimationPosition  0*4

#define EEPROM_1_ActColor              0x000000F0
#define EEPROM_1_ActColorPosition      1*4

#define EEPROM_1_ActAlarmTone          0x00000F00
#define EEPROM_1_ActAlarmTonePosition  2*4

#define EEPROM_1_ActAlarmVol           0x0000F000
#define EEPROM_1_ActAlarmVolPosition   3*4

#define EEPROM_1_ActAlarmTempo         0x000F0000
#define EEPROM_1_ActAlarmTempoPosition 4*4

#define EEPROM_1_ActBrightness         0x00F00000
#define EEPROM_1_ActBrightnessPosition 5*4

#define EEPROM_1_ActMusic              0x0F000000
#define EEPROM_1_ActMusicPosition      6*4

// Public functions
extern void     EEPROM_SystemBackup(void);

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
