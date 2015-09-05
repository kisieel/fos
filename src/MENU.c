#include <stdlib.h>
#include "stm32l1xx.h"

#include "main.h"

MENU *_1_menu;
MENU *_2_menu;
MENU *_3_menu;
MENU *_4_menu;
MENU *_5_menu;
MENU *_6_menu;
MENU *_7_menu;
MENU *_8_menu;
MENU *_9_menu;

MENU *_actual;

// 0 level
void menu_1_fun(unsigned int);
void menu_2_fun(unsigned int);
void menu_3_fun(unsigned int);
void menu_4_fun(unsigned int);
void menu_5_fun(unsigned int);
void menu_6_fun(unsigned int);
void menu_7_fun(unsigned int);
void menu_8_fun(unsigned int);
void menu_9_fun(unsigned int);

void MENU_init()
{
	_1_menu = malloc(sizeof(MENU));
	_2_menu = malloc(sizeof(MENU));
	_3_menu = malloc(sizeof(MENU));
	_4_menu = malloc(sizeof(MENU));
	_5_menu = malloc(sizeof(MENU));
	_6_menu = malloc(sizeof(MENU));
	_7_menu = malloc(sizeof(MENU));
	_8_menu = malloc(sizeof(MENU));
	_9_menu = malloc(sizeof(MENU));
	
// 0 level
	_1_menu->next = _2_menu;
	_1_menu->par = NULL;
	_1_menu->sub = NULL;
	_1_menu->menu_fun = menu_1_fun;
	
	_2_menu->next = _3_menu;
	_2_menu->par = NULL;
	_2_menu->sub = NULL;
	_2_menu->menu_fun = menu_2_fun;
	
	_3_menu->next = _4_menu;
	_3_menu->par = NULL;
	_3_menu->sub = NULL;
	_3_menu->menu_fun = menu_3_fun;
	
	_4_menu->next = _5_menu;
	_4_menu->par = NULL;
	_4_menu->sub = NULL;
	_4_menu->menu_fun = menu_4_fun;
	
	_5_menu->next = _6_menu;
	_5_menu->par = NULL;
	_5_menu->sub = NULL;
	_5_menu->menu_fun = menu_5_fun;
	
	_6_menu->next = _7_menu;
	_6_menu->par = NULL;
	_6_menu->sub = NULL;
	_6_menu->menu_fun = menu_6_fun;
	
	_7_menu->next = _8_menu;
	_7_menu->par = NULL;
	_7_menu->sub = NULL;
	_7_menu->menu_fun = menu_7_fun;
	
	_8_menu->next = _9_menu;
	_8_menu->par = NULL;
	_8_menu->sub = NULL;
	_8_menu->menu_fun = menu_8_fun;
	
	_9_menu->next = _1_menu;
	_9_menu->par = NULL;
	_9_menu->sub = NULL;
	_9_menu->menu_fun = menu_9_fun;
	
	_actual = _1_menu;
}

// Sygnalizacja brania- diody + buzzer
void menu_1_fun(unsigned int key)
{
	switch (key) {
//		case (KEY_1):
//			
//			ClrKeyb( KBD_LOCK );
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			_actual = _actual->next;
			break;
	}
}

// Wybor animacji brania
void menu_2_fun(unsigned int key)
{
	switch (key) {
//		case (KEY_1):
//			_BUZZER_single_beep();
//			ClrKeyb( KBD_LOCK );
//			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActAnimation) | (System.ActAnimation << EEPROM_1_ActAnimationPosition));
			_LED_set_color_list(4, 0);
			_LED_on();
		
			_actual = _actual->next;
			break;
	}
}

// Regulacja koloru diody
void menu_3_fun(unsigned int key)
{	
	switch (key) {
//		case (KEY_1):
//			_BUZZER_single_beep();
//			ClrKeyb( KBD_LOCK );
//		
//			if (System.ActColor == 0)
//				System.ActColor = led_colors_qnt - 1;
//			else
//				System.ActColor--;
//			_LED_set_color_list(4, System.ActColor);
//			_LED_on();
//			
//			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			if (System.ActColor == led_colors_qnt - 1)
				System.ActColor = 0;
			else
				System.ActColor++;
			_LED_set_color_list(5, System.ActColor);
			_LED_on();
			
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActColor) | (System.ActColor << EEPROM_1_ActColorPosition));
			_LED_set_color_list(4, 1);
			_LED_on();
		
			_actual = _actual->next;
			break;
	}
}

// Wybor jasnosci diod
void menu_4_fun(unsigned int key)
{
	
	switch (key) {
//		case (KEY_1):
//			_BUZZER_single_beep();
//			ClrKeyb( KBD_LOCK );
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActBrightness) | (System.ActBrightness << EEPROM_1_ActBrightnessPosition));
			_LED_set_color_list(4, 2);
			_LED_on();
		
			_actual = _actual->next;
			break;
	}
}

// Wybor tonu alarmu
void menu_5_fun(unsigned int key)
{
	_BUZZER_alarm_start();
	
	switch (key) {
//		case (KEY_1):
//			ClrKeyb( KBD_LOCK );
//		
//			if (System.ActAlarmTone == 0)
//				System.ActAlarmTone = buzzer_tones_qnt - 1;
//			else
//				System.ActAlarmTone--;
//			_BUZZER_alarm_set_tone_list(System.ActAlarmTone);
//			
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
		
			if (System.ActAlarmTone == buzzer_tones_qnt - 1)
				System.ActAlarmTone = 0;
			else
				System.ActAlarmTone++;
			_BUZZER_alarm_set_tone_list(System.ActAlarmTone);
			
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActAlarmTone) | (System.ActAlarmTone << EEPROM_1_ActAlarmTonePosition));
			_LED_set_color_list(4, 3);
			_LED_on();
		
			_actual = _actual->next;
			break;
	}
}

// Wybor glososci alarmu
void menu_6_fun(unsigned int key)
{
	_BUZZER_alarm_start();
	
	switch (key) {
//		case (KEY_1):
//			ClrKeyb( KBD_LOCK );
//		
//			if (System.ActAlarmVol == 0)
//				System.ActAlarmVol = buzzer_vols_qnt - 1;
//			else
//				System.ActAlarmVol--;
//			_BUZZER_alarm_set_vol_list(System.ActAlarmVol);
//		
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
		
			if (System.ActAlarmVol == buzzer_vols_qnt - 1)
				System.ActAlarmVol = 0;
			else
				System.ActAlarmVol++;
			_BUZZER_alarm_set_vol_list(System.ActAlarmVol);
		
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActAlarmVol) | (System.ActAlarmVol << EEPROM_1_ActAlarmVolPosition));
			_LED_set_color_list(4, 4);
			_LED_on();
		
			_actual = _actual->next;
			break;
	}
}

// Wybor tempa alarmu
void menu_7_fun(unsigned int key)
{
	_BUZZER_alarm_start();
	
	switch (key) {
//		case (KEY_1):
//			ClrKeyb( KBD_LOCK );
//			
//			if (System.ActAlarmTempo == 0)
//				System.ActAlarmTempo = buzzer_tempos_qnt - 1;
//			else
//				System.ActAlarmTempo--;
//			_BUZZER_alarm_set_tempo_list(System.ActAlarmTempo);
//		
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
		
			if (System.ActAlarmTempo == buzzer_tempos_qnt - 1)
				System.ActAlarmTempo = 0;
			else
				System.ActAlarmTempo++;
			_BUZZER_alarm_set_tempo_list(System.ActAlarmTempo);
		
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (EEPROM_32_read(EEPROM_ConfAddress1) & ~EEPROM_1_ActAlarmTempo) | (System.ActAlarmTempo << EEPROM_1_ActAlarmTempoPosition));
			_LED_set_color_list(4, 5);
			_LED_on();
			_BUZZER_alarm_stop();
		
			_actual = _actual->next;
			break;
	}
}

// 
void menu_8_fun(unsigned int key)
{
	
	switch (key) {
//		case (KEY_1):
//			_BUZZER_single_beep();
//			ClrKeyb( KBD_LOCK );
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			_actual = _actual->next;
		
			break;
	}
}

// 
void menu_9_fun(unsigned int key)
{
	
	switch (key) {
//		case (KEY_1):
//			_BUZZER_single_beep();
//			ClrKeyb( KBD_LOCK );
//			_actual = _actual->next;
//			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			_actual = _actual->next;
		
			break;
	}
}