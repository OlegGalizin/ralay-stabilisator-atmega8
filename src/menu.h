#if !defined(__MENU_H__)
#define __MENU_H__
#include <avr/io.h>
#include <avr/interrupt.h> 
#include <math.h>
#include <avr/eeprom.h>
#include <string.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <inttypes.h>


#define EV_KEY_TOUCH    0x01
#define EV_KEY_PRESSED  0x02 
#define EV_KEY_LONG     0x03
#define EV_KEY_REPEATE  0x04
#define EV_KEY_REALIZED 0x05
#define EV_FUNC_FIRST   0x07
#define EV_MASK         0x07

#define WAS_NO_EVENT        ((IE1 & WDTIE) == 0)
extern uint8_t Event;  /* 0xKKKKKAAA */
/* AAA - it is event key. Event can be defined by & operation with EV_MASK*/
/* KKKKKK - keys - one bit fpr one key. Up to 5 keys */

extern uint8_t MenuCounter;
/* Additional variable should be used in every code */

extern uint8_t EventQueue;


extern uint8_t CurrentFunc;
extern uint8_t EvCounter; // Press delay counter


/* Variable to change event handler */

typedef void (*MenuFunction_t)(void);
extern const MenuFunction_t FuncArray[]; // Array of functions of the menu

void MenuCheckEvent(void);
void MenuInit(void);

#define KEYPORT PIND

#define KEY1  _BV(PD5)  //S3
#define KEY2  _BV(PD7)  //S1
#define KEY3  _BV(PD6)  //S2
#define KEY_MASK (KEY1 | KEY2 | KEY3 )

#define KEY_UP 	        KEY3
#define KEY_ENTER 	KEY2
#define KEY_DOWN 	KEY1

#endif

