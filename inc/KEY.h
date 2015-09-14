#ifndef KEY__H
#define KEY__H
 
// SW_2, KEY_1   PA1
// SW_3, KEY_2   PA2

// KEY_1- minus,
// KEY_2- plus,
// KEY_3- akceptuj

#define KEY1      (1<<1)
#define KEY2      (1<<0)
#define KEY3      (1<<2)

#define KEY_1      KEY1
#define KEY_2      KEY2
#define KEY_3      KEY3

#define ANYKEY		 (KEY1 | KEY2 | KEY3)
#define KEY_MASK	 (KEY1 | KEY2 | KEY3)

#define KBD_LOCK	  1
#define KBD_NOLOCK	0

#define KBD_DEFAULT_ART	((void *)0)

void KEY_init(void);

void ClrKeyb( int lock );
unsigned int GetKeys( void );
unsigned int KeysTime( void );
unsigned int IsKeyPressed( unsigned int mask );
unsigned intIsKey( unsigned int mask );
void KeybLock( void );
void KeybSetAutoRepeatTimes( unsigned short * AutoRepeatTab );
void KeybProc( void );

#endif
