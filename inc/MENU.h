#ifndef __MENU_H
#define __MENU_H

typedef struct _menuitem
{
	struct _menuitem *next;
	struct _menuitem *prev;
	struct _menuitem *sub;
	struct _menuitem *par;
	void (*menu_fun)(unsigned int);
} MENU;

extern MENU *_actual;

void MENU_init(void);

// 0 level
void _menu_1_fun(unsigned int);
void _menu_2_fun(unsigned int);

#endif