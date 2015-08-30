#ifndef __MENU_H
#define __MENU_H

typedef struct _menuitem
{
	struct _menuitem *next;
//	struct _menuitem *prev;
	struct _menuitem *sub;
	struct _menuitem *par;
	void (*menu_fun)(unsigned int);
} MENU;

extern MENU *_actual;

void MENU_init(void);



#endif
