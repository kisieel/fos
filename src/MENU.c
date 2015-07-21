#include <stdlib.h>
#include "stm32l1xx.h"

#include "MENU.h"
#include "KEY.h"

MENU *_1_menu;
MENU *_2_menu;

MENU *_actual;

void MENU_init()
{
	_1_menu = malloc(sizeof(MENU));
	_2_menu = malloc(sizeof(MENU));
	
// 0 level
	_1_menu->next = _2_menu;
	_1_menu->prev = _2_menu;
	_1_menu->par = NULL;
	_1_menu->sub = NULL;
	_1_menu->menu_fun = _menu_1_fun;
	
	_2_menu->next = _1_menu;
	_2_menu->prev = _1_menu;
	_2_menu->par = NULL;
	_2_menu->sub = NULL;
	_2_menu->menu_fun = _menu_2_fun;
	
	_actual = _1_menu;
}

void _menu_1_fun(unsigned int key)
{
	switch (key) {
		case (KEY_1):
			ClrKeyb( KBD_LOCK );
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
			break;
		case (KEY_2):
			ClrKeyb( KBD_LOCK );
			break;
	}
}