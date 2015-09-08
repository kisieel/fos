#include <stdlib.h>
#include "main.h"

MENU *_1_menu;
MENU *_2_menu;
MENU *_3_menu;
MENU *_4_menu;
MENU *_5_menu;
MENU *_6_menu;
MENU *_7_menu;
MENU *_8_menu;

#define MENU_LED_Menu_2  4
#define MENU_LED_Color   3
#define MENU_LED_Menu_1  2

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
	
	_8_menu->next = _1_menu;
	_8_menu->par = NULL;
	_8_menu->sub = NULL;
	_8_menu->menu_fun = menu_8_fun;
	
	_actual = _1_menu;
}

// Sygnalizacja brania- diody + buzzer
void menu_1_fun(unsigned int key)
{
	HALL_Data.HuntTime = TRUE;
	
	switch (key) {
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			_LED_set_color_list(MENU_LED_Menu_1, 0);
			_LED_set_color_list(MENU_LED_Menu_2, 0);
			_LED_on();
			HALL_Data.HuntTime = FALSE;

#ifdef USART_debug
			USART_send("-2- LED animation setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor animacji brania
void menu_2_fun(unsigned int key)
{	
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	_LED_animate(led_animate_mode_single);
	switch (key) {
		case (KEY_2):
			_BUZZER_single_beep();
		
			if (System.ActAnimation >= led_amimations_qnt - 1)
				System.ActAnimation = 0;
			else
				System.ActAnimation++;
			_LED_change_animate_list(System.ActAnimation);
			
#ifdef USART_debug
			USART_send("\tLED animation #");
			USART_write_buf(System.ActAnimation, DEC);
			USART_send(".\n");
#endif
			
			ClrKeyb( KBD_LOCK );
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			_LED_animate_off();
		
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActAnimation) | (System.ActAnimation << EEPROM_1_ActAnimationPosition));
			_LED_set_color_list(MENU_LED_Color, System.ActColor);
			_LED_set_color_list(MENU_LED_Menu_1, 1);
			_LED_set_color_list(MENU_LED_Menu_2, 1);
			_LED_on();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);
		
#ifdef USART_debug
			USART_send("-3- LED color setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Regulacja koloru diody
void menu_3_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	_LED_blink_on(MENU_LED_Color);

	switch (key) {
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			if (System.ActColor == led_colors_qnt - 1)
				System.ActColor = 0;
			else
				System.ActColor++;
			_LED_set_color_list(MENU_LED_Color, System.ActColor);
			_LED_on();
			
#ifdef USART_debug
			USART_send("\tLED color #");
			USART_write_buf(System.ActColor, DEC);
			USART_send(".\n");
#endif
			
			break;
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );

			_LED_blink_off(MENU_LED_Color);	
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActColor) | (System.ActColor << EEPROM_1_ActColorPosition));
			_LED_set_color_list(MENU_LED_Menu_1, 2);
			_LED_set_color_list(MENU_LED_Menu_2, 2);
			_LED_on();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);

#ifdef USART_debug
			USART_send("-4- LED brightness setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor jasnosci diod
void menu_4_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	switch (key) {
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
		
			if (System.ActBrightness == led_brightness_qnt - 1)
				System.ActBrightness = 0;
			else
				System.ActBrightness++;
			_LED_change_brightness_limit_list(System.ActBrightness);
			_LED_on();
		
#ifdef USART_debug
			USART_send("\tLED brightness #");
			USART_write_buf(System.ActBrightness, DEC);
			USART_send(".\n");
#endif
			
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActBrightness) | (System.ActBrightness << EEPROM_1_ActBrightnessPosition));
			_LED_set_color_list(MENU_LED_Menu_1, 3);
			_LED_set_color_list(MENU_LED_Menu_2, 3);
			_LED_on();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);

#ifdef USART_debug
			USART_send("-5- Alarm tone setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor tonu alarmu
void menu_5_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	_BUZZER_alarm_start();
	
	switch (key) {
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
		
			if (System.ActAlarmTone == buzzer_tones_qnt - 1)
				System.ActAlarmTone = 0;
			else
				System.ActAlarmTone++;
			_BUZZER_alarm_set_tone_list(System.ActAlarmTone);
			
#ifdef USART_debug
			USART_send("\tAlarm tone #");
			USART_write_buf(System.ActAlarmTone, DEC);
			USART_send(".\n");
#endif
			
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActAlarmTone) | (System.ActAlarmTone << EEPROM_1_ActAlarmTonePosition));
			_LED_set_color_list(MENU_LED_Menu_1, 4);
			_LED_set_color_list(MENU_LED_Menu_2, 4);
			_LED_on();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);

#ifdef USART_debug
			USART_send("-6- Alarm volume setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor glososci alarmu
void menu_6_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	_BUZZER_alarm_start();
	
	switch (key) {
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			if (System.ActAlarmVol == buzzer_vols_qnt - 1)
				System.ActAlarmVol = 0;
			else
				System.ActAlarmVol++;
			_BUZZER_alarm_set_vol_list(System.ActAlarmVol);
		
#ifdef USART_debug
			USART_send("\tAlarm volume #");
			USART_write_buf(System.ActAlarmVol, DEC);
			USART_send(".\n");
#endif
			
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActAlarmVol) | (System.ActAlarmVol << EEPROM_1_ActAlarmVolPosition));
			_LED_set_color_list(MENU_LED_Menu_1, 5);
			_LED_set_color_list(MENU_LED_Menu_2, 5);
			_LED_on();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);

#ifdef USART_debug
			USART_send("-7- Alarm tempo setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor tempa alarmu
void menu_7_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	_BUZZER_alarm_start();
	
	switch (key) {
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
		
			if (System.ActAlarmTempo == buzzer_tempos_qnt - 1)
				System.ActAlarmTempo = 0;
			else
				System.ActAlarmTempo++;
			_BUZZER_alarm_set_tempo_list(System.ActAlarmTempo);
		
#ifdef USART_debug
			USART_send("\tAlarm tempo #");
			USART_write_buf(System.ActAlarmTempo, DEC);
			USART_send(".\n");
#endif
			
			break;
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
		
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActAlarmTempo) | (System.ActAlarmTempo << EEPROM_1_ActAlarmTempoPosition));
			_LED_set_color_list(MENU_LED_Menu_1, 6);
			_LED_set_color_list(MENU_LED_Menu_2, 6);
			_LED_on();
			_BUZZER_alarm_stop();
			_BUZZER_play_music(System.ActMusic);
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);

#ifdef USART_debug
			USART_send("-8- Music setting mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}

// Wybor muzyki
void menu_8_fun(unsigned int key)
{
	uint32_t buffer;
	uint8_t buffer_send[4];
	
	switch (key) {
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			
			if (System.ActMusic == buzzer_musics_qnt - 1)
				System.ActMusic = 0;
			else
				System.ActMusic++;
			_BUZZER_play_music(System.ActMusic);
			
#ifdef USART_debug
			USART_send("\tMusic #");
			USART_write_buf(System.ActMusic, DEC);
			USART_send(".\n");
#endif
			
			break;
		
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			
			EEPROM_32_write(EEPROM_ConfAddress1, (buffer & ~EEPROM_1_ActAlarmTempo) | (System.ActAlarmTempo << EEPROM_1_ActAlarmTempoPosition));
			_LED_set_color(MENU_LED_Menu_1, 0, 0, 0);
			_LED_set_color(MENU_LED_Menu_2, 0, 0, 0);
			_LED_on();
			_BUZZER_stop_music();
		
			buffer = EEPROM_32_read(EEPROM_ConfAddress1);
			buffer_send[0] = (buffer & 0x000000FF) >> 0;
			buffer_send[1] = (buffer & 0x0000FF00) >> 8;
			buffer_send[2] = (buffer & 0x00FF0000) >> 16;
			buffer_send[3] = (buffer & 0xFF000000) >> 24;
//			RFM69W_sendWithRetry(0x00, buffer_send, 4, 10, 10);
			
#ifdef USART_debug
			USART_send("-1- Hunter mode.\n");
#endif
		
			_actual = _actual->next;
			break;
	}
}