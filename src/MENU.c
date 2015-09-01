#include <stdlib.h>
#include "stm32l1xx.h"

#include "main.h"

MENU *_1_menu;
MENU *_2_menu;
MENU *_3_menu;
MENU *_4_menu;

MENU *_actual;

// 0 level
void menu_1_fun(unsigned int);
void menu_2_fun(unsigned int);
void menu_3_fun(unsigned int);
void menu_4_fun(unsigned int);

void MENU_init()
{
	_1_menu = malloc(sizeof(MENU));
	_2_menu = malloc(sizeof(MENU));
	_3_menu = malloc(sizeof(MENU));
	_4_menu = malloc(sizeof(MENU));
	
// 0 level
	_1_menu->next = _2_menu;
//	_1_menu->prev = _4_menu;
	_1_menu->par = NULL;
	_1_menu->sub = NULL;
	_1_menu->menu_fun = menu_1_fun;
	
	_2_menu->next = _3_menu;
//	_2_menu->prev = _1_menu;
	_2_menu->par = NULL;
	_2_menu->sub = NULL;
	_2_menu->menu_fun = menu_2_fun;
	
	_3_menu->next = _4_menu;
//	_3_menu->prev = _2_menu;
	_3_menu->par = NULL;
	_3_menu->sub = NULL;
	_3_menu->menu_fun = menu_3_fun;
	
	_4_menu->next = _1_menu;
//	_4_menu->prev = _3_menu;
	_4_menu->par = NULL;
	_4_menu->sub = NULL;
	_4_menu->menu_fun = menu_4_fun;
	
	_actual = _1_menu;
}

// Sygnalizacja brania- diody + buzzer
void menu_1_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
	}
}

// Regulacja tonu
void menu_2_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
	}
}

// Regulacja koloru diod
void menu_3_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
	}
}

// Tryb testowania zasiegu
void menu_4_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			_BUZZER_single_beep();
			ClrKeyb( KBD_LOCK );
			break;
	}
}
