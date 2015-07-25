#include <stdlib.h>
#include "stm32l1xx.h"

#include "MENU.h"
#include "KEY.h"

MENU *_1_menu;
MENU *_2_menu;
MENU *_3_menu;
MENU *_4_menu;

MENU *_actual;

void _MENU_init()
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
	_1_menu->menu_fun = _menu_1_fun;
	
	_2_menu->next = _3_menu;
//	_2_menu->prev = _1_menu;
	_2_menu->par = NULL;
	_2_menu->sub = NULL;
	_2_menu->menu_fun = _menu_2_fun;
	
	_3_menu->next = _4_menu;
//	_3_menu->prev = _2_menu;
	_3_menu->par = NULL;
	_3_menu->sub = NULL;
	_3_menu->menu_fun = _menu_3_fun;
	
	_4_menu->next = _1_menu;
//	_4_menu->prev = _3_menu;
	_4_menu->par = NULL;
	_4_menu->sub = NULL;
	_4_menu->menu_fun = _menu_4_fun;
	
	_actual = _1_menu;
}

void _menu_1_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
	}
}

void _menu_2_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
	}
}

void _menu_3_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
	}
}

void _menu_4_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
			_actual = _actual->next;
			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
	}
}
