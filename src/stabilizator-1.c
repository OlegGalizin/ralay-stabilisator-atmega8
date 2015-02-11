//#define DEBUG_EE
//#define DISPLAY_OA
#define DISPLAY_N3310


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
#if defined(DISPLAY_N3310)
#include "n3310.h"
#endif


#define DEFAULT_MAX 240 //макс напр сети по умолч
#define DEFAULT_MIN 190 //мин напр сети по умолч
#define DEFAULT_CONTRAST 60 // контраст Н3310 индикатора по умолч


#include "menu.h"

#define F_CPU 8000000UL  

#define Coeff (16/1.56)  //Коэффициент пропорциональности напряжения
#define RmsP2(V) ((unsigned int)(((double)V)*V/Coeff/Coeff)) // макрос перевода вольт в попугаи

#define PinOn _BV(PB1)   // пин реле включения нагрузки
#define PinOnPORT PORTB

#define PinAlarm _BV(PC1)
#define PortAlarm PORTC

#if defined(DISPLAY_N3310)
#define  PortLight PORTD
#define  PinLight  _BV(PD6)
uint16_t  CounterLight;
#endif

uint16_t ResetCounter;

static  void check_keys(void);

unsigned char CurrentDigit; // Текущая отображаемая цифра
volatile uint8_t Display[3]; //отображаемые цифры ( диапазон 0 - 9)

#if defined(DISPLAY_N3310)
uint8_t DisplayOld[3]; //отображаемые цифры ( диапазон 0 - 9)
#endif

uint8_t CurrentLast = 0;//Где в массиве лежит последнее значение

EEMEM uint16_t ee_U_max = DEFAULT_MAX;		//максимальное наряжение сети
EEMEM uint16_t ee_U_min = DEFAULT_MIN;		//минимальное напряжение сети
EEMEM uint16_t ee_zadergka = 100;	//времья задержки на включение
EEMEM uint16_t ee_filter = 0;		//времья нечувствительности колебаний сетевого напряжения
#if defined(DISPLAY_N3310)
EEMEM uint8_t ee_contrast = DEFAULT_CONTRAST;
uint8_t Contrast;
#endif /* DISPLAY_N3310 */

volatile uint16_t U_max;  //максимальное наряжение сети
volatile uint16_t U_min; //минимальное напряжение сети
volatile uint16_t U_maxUE;  //максимальное наряжение сети
volatile uint16_t U_minUE; //минимальное напряжение сети
volatile uint16_t zadergka; // Задержка включения нагрузки после аварии
volatile uint16_t t_filter; // времья нечуствительности колебаний сетевого напряжения

volatile int Timer = 0; // Таймер задержки включения: 50 периодов  - 1 сек , 250 - 5 сек
volatile int tm1 = 0;
volatile char key_code = 0;


#if !defined(DISPLAY_N3310)
#define SHIFT595 _BV(PD7)
#define DATA595  _BV(PD5)
#define STORE595 _BV(PD6)

#define SHIFT595PORT PORTD
#define DATA595PORT  PORTD
#define STORE595PORT PORTD
#define DIG1 _BV(PD0)
#define DIG2 _BV(PD1)
#define DIG3 _BV(PD2)
#define DIG4 _BV(PD4)
#define DIGPORT PORTD
#endif


#define SEG_A 0x04
#define SEG_B 0x01
#define SEG_C 0x40
#define SEG_D 0x10
#define SEG_E 0x08
#define SEG_F 0x02
#define SEG_G 0x80
#define SEG_H 0x20
#define SEG_DOT SEG_H

const unsigned char digit[11]=
{SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F, //0
 SEG_B|SEG_C, //1
 SEG_A|SEG_B|SEG_D|SEG_E|SEG_G, //2
 SEG_A|SEG_B|SEG_C|SEG_D|SEG_G, //3
 SEG_B|SEG_C|SEG_F|SEG_G, //4
 SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,//5
 SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, //6
 SEG_A|SEG_B|SEG_C,//7
 SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, //8
 SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G,//9
 0x00,}; // откл.  

#if !defined(DISPLAY_N3310)
#if !defined(DISPLAY_OA)
const unsigned char DigitLine[3] = {~DIG3, ~DIG1, ~DIG2 }; //Какую цифру отображать                                                                  
#else
const unsigned char DigitLine[3] = {DIG3, DIG1, DIG2 }; //Какую цифру отображать                                                                  
#endif
#endif /* DISPLAY_N3310 */

long Summ; // Накопитель квадратов
volatile long LastSumm; // Здесь накопленный результат передается в main
int Counts; //Накопитель количества измерений
volatile int LastCounts;//количество измерений передается в main. Индикатор окончания измерений
unsigned int  MinWv = 0xFFFF;	// Минимальное взвешенное значение на полупериоде
unsigned int  MaxWv = 0; 		// Максимальное взвешенное значение на полупериоде
unsigned int ImmediateValue; 	// для попугаев

//последние измеренные значения. Хранится 2 набора по 8 штук
int  LastValues[16] = {0x400,0x400,0x400,0x400,0x400,0x400,0x400,0x400,
                       0x400,0x400,0x400,0x400,0x400,0x400,0x400,0x400};

int8_t DisplayCounter; //Счетчик времни отображения индикации
 
static void Decoder_display(int DataForLed) //Преобразование числа в строку для отображения
{
  int Div = 100;
  uint8_t* Pointer = Display;
  char i = 3;
  char OnFlag = 0;

  while (i)
  {
     uint8_t Dig = DataForLed/Div;

     if ( Dig == 0 && OnFlag == 0 && i != 1)
     {
       *Pointer = 0; //отключить незначущийся ноль
     }
     else
     {
       *Pointer = digit[Dig];
       OnFlag = 1;
     }
     Pointer++;
     i--;
     DataForLed = DataForLed%Div;
     Div = Div/10;
  } //while
}

uint8_t RelayCounter;

static void Regulator (void)
{
  if ( ImmediateValue > (U_maxUE) || 
       ImmediateValue < (U_minUE) )
  {
    if ( RelayCounter < t_filter ) //Задержка отключения
    {
      RelayCounter++;
      return;	  
    }
    goto D0;
  }
  RelayCounter=0;


  if ( Timer < zadergka ) // 5 сек. задержка: 50 периодов=1 сек, 250 = 5 сек
  {
    Timer = Timer + 1;
    return;
  }
   

  PinOnPORT |= PinOn;    // Включить выход
  PortAlarm &= ~PinAlarm;
//  {
//    uint16_t Out = (uint16_t)(sqrt(ImmediateValue)*Coeff); //Out moving average value

//    Decoder_display(Out);
//    Display[2] |= SEG_DOT;
//    DisplayCounter = -100;
//  }
  return;

D0: // все выкл
  Timer = 0;         // Сброс таймера задержки включения
  RelayCounter = 0;
  PinOnPORT &= ~PinOn;   // ОТКЛЮЧИТЬ ВСЕ РЕЛЕ И НАГРУЗКУ!!!
  PortAlarm |= PinAlarm; 
}


static void Init(void)
{
// Input/Output Ports initialization
// Port B initialization
	PORTB=0x00;
	DDRB=(PinOn|PinAlarm);

// Port C initialization
	PORTC=~_BV(PC0); // PC0 - АЦП без подтяжки
	DDRC=0x02; // ALARM is out

// Port D initialization
#if !defined(DISPLAY_N3310)
#if !defined(DISPLAY_OA)
	PORTD=0xFF;                                                         
#else
	PORTD=0x00;                                                         
#endif
#else
	PORTD=0x00;                                                         
#endif
	DDRD=0xFF; // All out
	
// Timer/Counter 0 initialization
	TCCR0=0x00;
	TCNT0=0;

// Timer/Counter 1 initialization
	TCCR1A=0x00;
	TCCR1B=0x00;
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

// Timer/Counter 2 initialization
	ASSR=0x00;
	TCCR2=0x00;
	TCNT2=0x00;
	OCR2=0x00;

// External Interrupt(s) initialization
	MCUCR=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK=0x00;

// Analog Comparator initialization
	ACSR=0x80;
	SFIOR=0x00;

// ADC initialization
// ADC Clock frequency: 125,000 kHz
// ADC Voltage Reference: Int., cap. on AREF
	ADMUX=/*_BV(REFS1)|*/_BV(REFS0); /* --2.56v--- reference & ADC0 */
	ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADIE)|_BV(ADPS1)|_BV(ADPS2);  // div64
// ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADIE)|_BV(ADPS0)|_BV(ADPS2);  // div32

#if !defined(DEBUG_EE)
	U_max = eeprom_read_word( &ee_U_max);
        if ( U_max == 0xFFFF )
#endif
        {
          U_max = DEFAULT_MAX;
        }
        U_maxUE = RmsP2(U_max);

#if !defined(DEBUG_EE)
	U_min = eeprom_read_word( &ee_U_min);
        if ( U_min == 0xFFFF )
#endif
        {
          U_min = DEFAULT_MIN;
        }
        U_minUE = RmsP2(U_min);
		  
#if !defined(DEBUG_EE)
	zadergka = eeprom_read_word( &ee_zadergka);
        if ( zadergka == 0xFFFF )
#endif
        {
          zadergka = 100;
        }
	
#if !defined(DEBUG_EE)
	t_filter = eeprom_read_word( &ee_filter);
        if ( t_filter == 0xFFFF )
#endif
        {
          t_filter = 0;
        }
#if defined(DISPLAY_N3310)
#if !defined(DEBUG_EE)
	Contrast = eeprom_read_byte( &ee_contrast);
        if ( Contrast == 0xFF )
#endif
          Contrast = 60;  
	      LcdInit(Contrast);
        LcdClear();
#endif /*DISPLAY_N3310 */

}

long AvSum16=580000L;  // это усредненное значение
uint16_t  AvCount16 = 200*16;// Усредненное количество отсчетов
static uint8_t PrevFunc;       // Previous function.

#define MAIN_FUNCTION 0
#define HERTZ_FUNCTION 1
#define MENU_FUNCTION 2
#define HI_FUNCTION 3
#define LO_FUNCTION 4
#define DELAY_FUNCTION 5
#define FILTER_FUNCTION 6
#define SAVED_FUNCTION 7
#define CONTRAST_FUNCTION 8

int16_t Value;

#if defined(DISPLAY_N3310)
void ContrastFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    goto redraw;
  }
  if ( (Event & KEY_MASK) == KEY_ENTER )
  {
    if ( ((Event & EV_MASK) == EV_KEY_LONG) || 
         ((Event & EV_MASK) == EV_KEY_REALIZED))
    {
      if ( (Event & EV_MASK) == EV_KEY_LONG)
      {
        eeprom_write_byte( &ee_contrast, Contrast);
        CurrentFunc = SAVED_FUNCTION;
        return;
      }
        CurrentFunc = MENU_FUNCTION;
      return;
    }
    return;
  }

  if ( (Event & EV_MASK) == EV_KEY_PRESSED ||
       (Event & EV_MASK) == EV_KEY_REPEATE)
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Contrast < 90 )
        Contrast++;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Contrast > 10 )
        Contrast--;
    }
    LcdContrast(Contrast);
  }

redraw:
  if (DisplayCounter >= 0 )
  { 
    Decoder_display(Contrast);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;

}
#endif /*DISPLAY_N3310 */



void SavedFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    Value = 0;
    goto redraw;
  }
  Value++;
  if ( Value > 200 )
  {
    CurrentFunc = MENU_FUNCTION;
    return;
  }
redraw:
  Display[0] = SEG_A|SEG_C|SEG_D|SEG_F|SEG_G; // S
  Display[1] = SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G; // A
  Display[2] = 0;
}

void HiFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    Value = U_max;
    DisplayCounter = 16;
    goto redraw;
  }

  if ( (Event & KEY_MASK) == KEY_ENTER )
  {
    if ( ((Event & EV_MASK) == EV_KEY_LONG) || 
         ((Event & EV_MASK) == EV_KEY_REALIZED))
    {
      U_max = Value;
      U_maxUE = RmsP2(U_max);
      if ( (Event & EV_MASK) == EV_KEY_LONG)
      {
        eeprom_write_word( &ee_U_max, U_max);
        CurrentFunc = SAVED_FUNCTION;
        return;
      }
        CurrentFunc = MENU_FUNCTION;
      return;
    }
    return;
  }

  if ( (Event & EV_MASK) == EV_KEY_PRESSED ||
       (Event & EV_MASK) == EV_KEY_REPEATE)
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Value < 500 )
        Value++;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Value > 220 )
        Value--;
    }
  }

redraw:
  if (DisplayCounter >= 0 )
  { 
    Decoder_display(Value);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
}


void LoFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    Value = U_min;
    DisplayCounter = 16;
    goto redraw;
  }
  if ( (Event & KEY_MASK) == KEY_ENTER )
  {
    if ( ((Event & EV_MASK) == EV_KEY_LONG) || 
         ((Event & EV_MASK) == EV_KEY_REALIZED))
    {
      U_min = Value;
      U_minUE = RmsP2(U_min);
      if ( (Event & EV_MASK) == EV_KEY_LONG)
      {
        eeprom_write_word( &ee_U_min, U_min);
        CurrentFunc = SAVED_FUNCTION;
        return;
      }
        CurrentFunc = MENU_FUNCTION;
      return;
    }
    return;
  }

  if ( (Event & EV_MASK) == EV_KEY_PRESSED ||
       (Event & EV_MASK) == EV_KEY_REPEATE)
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Value < 220 )
        Value++;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Value > 100 )
        Value--;
    }
  }

redraw:
  if (DisplayCounter >= 0 )
  { 
    Decoder_display(Value);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
}


void DelayFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    Value = zadergka/50;
    DisplayCounter = 16;
    goto redraw;
  }

  if ( (Event & KEY_MASK) == KEY_ENTER )
  {
    if ( ((Event & EV_MASK) == EV_KEY_LONG) || 
         ((Event & EV_MASK) == EV_KEY_REALIZED))
    {
      zadergka = Value*50;
      if ( (Event & EV_MASK) == EV_KEY_LONG)
      {
        eeprom_write_word( &ee_zadergka, zadergka);
        CurrentFunc = SAVED_FUNCTION;
        return;
      }
      CurrentFunc = MENU_FUNCTION;
      return;
    }
    return;
  }


  if ( (Event & EV_MASK) == EV_KEY_PRESSED )
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Value < 640 )
        Value++;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Value > 0 )
        Value--;
    }
  }
  else
  if ( (Event & EV_MASK) == EV_KEY_REPEATE)
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Value < 640 )
        Value = Value + 10;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Value > 10 )
        Value = Value - 10;
    }
  }


redraw:
  if (DisplayCounter >= 0 )
  { 
    Decoder_display(Value);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
    
}

void FilterFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    Value = t_filter;
    DisplayCounter = 16;
    goto redraw;
  }

  if ( (Event & KEY_MASK) == KEY_ENTER )
  {
    if ( ((Event & EV_MASK) == EV_KEY_LONG) || 
         ((Event & EV_MASK) == EV_KEY_REALIZED))
    {
      t_filter = Value;
      if ( (Event & EV_MASK) == EV_KEY_LONG)
      {
        eeprom_write_word( &ee_filter, t_filter);
        CurrentFunc = SAVED_FUNCTION;
        return;
      }
      CurrentFunc = MENU_FUNCTION;
      return;
    }
    return;
  }

  if ( (Event & EV_MASK) == EV_KEY_PRESSED ||
       (Event & EV_MASK) == EV_KEY_REPEATE)
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      if ( Value < 999 )
        Value++;
    }
    else if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      if ( Value > 0 )
        Value--;
    }
  }

redraw:
  if (DisplayCounter >= 0 )
  { 
    Decoder_display(Value);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
}

void MainFunction(void)
{
  int Out;

  if (Event == EV_FUNC_FIRST)
  {
    DisplayCounter = 16;
    goto redraw;
  }
  if ( (Event & EV_MASK) == EV_KEY_PRESSED )
  {
    if ( (Event & KEY_MASK) == KEY_UP )
    {
      CurrentFunc = HERTZ_FUNCTION;
      return;
    }
#if defined(DISPLAY_N3310)
    if ( (Event & KEY_MASK) == KEY_DOWN )
    {
      PortLight |= PinLight;
      CounterLight=3000; // 1 min
      return;
    }
#endif
    MenuCounter = 0;
    CurrentFunc = MENU_FUNCTION;
    return;
  }

redraw:
  Out = (int)(sqrt((double)AvSum16/AvCount16)*Coeff); //Out moving average value

  if (DisplayCounter > 15 )
  { 
    Decoder_display(Out);
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
#if defined(DISPLAY_N3310)
  if (CounterLight)
    CounterLight--;
  else
    PortLight &= ~PinLight;
#endif
}

void HertzFunction(void)
{
  int Out;

  if (Event == EV_FUNC_FIRST)
  {
    DisplayCounter = 16;
    goto redraw;
  }
  if ( (Event & EV_MASK) == EV_KEY_PRESSED )
  {
    CurrentFunc = MAIN_FUNCTION;
    return;
  }

redraw:
  Out = (125000l*16*10/13)/AvCount16;

  if (DisplayCounter > 15 )
  { 
    Decoder_display(Out);
    Display[1] |= SEG_DOT;
    DisplayCounter = 0;
  }
  else
    DisplayCounter++;
}


static void MenuFunction(void)
{
  if (Event == EV_FUNC_FIRST)
  {
    if ( MenuCounter > 4 )
      MenuCounter = 0;
    goto redraw;
  }

  if ( (Event & EV_MASK) == EV_KEY_PRESSED )
  {
    switch (Event & KEY_MASK)
    {
      case KEY_UP:
#if defined(DISPLAY_N3310)
        if ( MenuCounter < 5 )
#else
        if ( MenuCounter < 4 )
#endif
          MenuCounter++;
        else
          MenuCounter = 0;
        break;
      case KEY_DOWN:
        if ( MenuCounter > 0 )
          MenuCounter--;
        else
#if defined(DISPLAY_N3310)
          MenuCounter = 5;
#else
          MenuCounter = 4;
#endif
        break;
      case KEY_ENTER:
         switch (MenuCounter)
         {
           case 1:
             CurrentFunc = HI_FUNCTION;
             return;
           case 2:
             CurrentFunc = LO_FUNCTION;
             return;
           case 3:
             CurrentFunc = DELAY_FUNCTION;
             return;
           case 4:
             CurrentFunc = FILTER_FUNCTION;
             return;
#if defined(DISPLAY_N3310)
           case 5:
             CurrentFunc = CONTRAST_FUNCTION;
             return;
#endif /*DISPLAY_N3310 */
         }
         CurrentFunc = MAIN_FUNCTION;
         return;
    }
  }

redraw:
  Display[0] = 0;
  Display[1] = 0;
  Display[2] = 0;
  switch ( MenuCounter )
  {
    case 0:
     Display[0] = SEG_E|SEG_G; // r
     Display[1] = SEG_A|SEG_D|SEG_E|SEG_F|SEG_G; // E
     break;
    case 1:
     Display[0] = SEG_B|SEG_C|SEG_E|SEG_F|SEG_G; // H
     Display[1] = SEG_E; // i
     break;
    case 2:
     Display[0] = SEG_D|SEG_E|SEG_F; // L
     Display[1] = SEG_C|SEG_D|SEG_E|SEG_G; // o
     break;
    case 3:
     Display[0] = SEG_B|SEG_C|SEG_D|SEG_E|SEG_G; // d
     Display[1] = SEG_A|SEG_D|SEG_E|SEG_F|SEG_G; // E
     break;
    case 4:
     Display[0] = SEG_A|SEG_E|SEG_F|SEG_G; // F
     Display[1] = SEG_E; // i
     break;
#if defined (DISPLAY_N3310)
    case 5:
     Display[0] = SEG_A|SEG_D|SEG_E|SEG_F; // C
     Display[1] = SEG_C|SEG_D|SEG_E|SEG_G; // o
     break;
#endif /*DISPLAY_N3310 */
  }
}


const MenuFunction_t FuncArray[] = {MainFunction, 
  HertzFunction,MenuFunction, HiFunction, LoFunction,
  DelayFunction,FilterFunction,SavedFunction
#if defined (DISPLAY_N3310)
  ,ContrastFunction
#endif /*DISPLAY_N3310 */
  }; // Array of functions of the menu



#if defined(DISPLAY_N3310)

void OutVBar(uint8_t xoff, uint8_t yoff, uint8_t data)
{
  uint8_t i;

  LcdGotoXY(xoff, yoff);
  for(i=0; i < 6; i++)
    LcdSend(data, LCD_DATA);

  LcdGotoXY(xoff, yoff+1);
  for(i=0; i < 6; i++)
    LcdSend(data, LCD_DATA);
}

void Draw(uint8_t dig)
{
  uint8_t Off = dig*28;
  uint8_t Char = Display[dig];
  uint8_t i;
  uint8_t Out;
//  LcdChr ( Y_POSITION*1+X_POSITION*1+13, "Hello world" );
  
  if (!(Char & SEG_A))
  {
    Out = 0x00;
    LcdGotoXY(Off, 0);
    for(i=0; i < 24; i++)
      LcdSend(Out, LCD_DATA);
  }


  if (!(Char & SEG_G))
  {
    Out = 0x00;
    LcdGotoXY(Off+6, 2);
    for(i=0; i < 12; i++)
      LcdSend(Out, LCD_DATA);
  }

  if (!(Char & SEG_D))
  {
    Out = 0x00;
    LcdGotoXY(Off, 4);
    for(i=0; i < 24; i++)
      LcdSend(Out, LCD_DATA);
  }



  if (!(Char & SEG_B) )
  {
    Out=0x00;
    OutVBar(Off+18, 0, Out);
  }
  
  if (!(Char & SEG_C))
  {
    Out=0x00;
    OutVBar(Off+18, 3, Out);
  }

  if (!(Char & SEG_E))
  {
    Out=0x00;
    OutVBar(Off, 3, Out);
  }

  if (!(Char & SEG_F))
  {
    Out=0x00;
    OutVBar(Off, 0, Out);
  }
 

  if (Char & SEG_A)
  {
    Out = 0x3F;
    LcdGotoXY(Off, 0);
    for(i=0; i < 24; i++)
      LcdSend(Out, LCD_DATA);
  }
  
  if (Char & SEG_G )
  {
    Out = 0x7e;
    LcdGotoXY(Off+6, 2);
    for(i=0; i < 12; i++)
      LcdSend(Out, LCD_DATA);
  }

  if ( (Char & SEG_G) )
  {
    Out = 0x7e;
    if (Char & SEG_F )
      Out |= 0x01;
    if (Char & SEG_E )
      Out |= 0x80;
  }
  else
  {
    Out = 0x00;
    if (Char & SEG_F )
      Out |= 0xFE;
    if (Char & SEG_E )
      Out |= 0x7F;
  }
  LcdGotoXY(Off, 2);
  for(i=0; i < 6; i++)
    LcdSend(Out, LCD_DATA);


  if ( (Char & SEG_G) )
  {
    Out = 0x7e;
    if (Char & SEG_B )
      Out |= 0x01;
    if (Char & SEG_C )
      Out |= 0x80;
  }
  else
  {
    Out = 0x00;
    if (Char & SEG_B )
      Out |= 0x7F;
    if (Char & SEG_C )
      Out |= 0xFE;
  }
  LcdGotoXY(Off+18, 2);
  for(i=0; i < 6; i++)
    LcdSend(Out, LCD_DATA);


  if (Char & SEG_D )
  {
    Out = 0xfc;
    LcdGotoXY(Off, 4);
    for(i=0; i < 24; i++)
      LcdSend(Out, LCD_DATA);
  }

  if (Char & SEG_B )
  {
    Out=0xFF;
    OutVBar(Off+18, 0, Out);
  }
  if (Char & SEG_C )
  {
    Out=0xFF;
    OutVBar(Off+18, 3, Out);
  }

  if (Char & SEG_E )
  {
    Out=0xFF;
    OutVBar(Off, 3, Out);
  }
  if (Char & SEG_F )
  {
    Out=0xFF;
    OutVBar(Off, 0, Out);
  }
  if (Char & SEG_H )
    Out=0x7E;
  else
    Out=0x00;
  LcdGotoXY(Off+22, 5);
  for(i=0; i < 4; i++)
    LcdSend(Out, LCD_DATA);
}
#endif


//*********************************************************

uint8_t RedrawFlag;

void main(void)
{
  // Declare your local variables here
  Init();

  // Global enable interrupts
  sei();

  while (1)
  {
    if (LastCounts != 0 ) // The voltage sum was calculated in the interrupt handler
    {
      long EndSum;
      int EndCount;

      cli(); // копировать намерянные величины без риска их изменения
      EndSum = LastSumm;
      EndCount = LastCounts;
      LastCounts = 0;
      sei(); // измерения за полпериода сохранены и могут сохраняться снова

      ImmediateValue = EndSum/256/EndCount; 
      Regulator();

      AvSum16 = AvSum16 + EndSum/256 - AvSum16/16 - 1;//взвешенное среднее по 4 точкам
      AvCount16 = AvCount16 + EndCount - AvCount16/16;// аналогично для количества

      check_keys();

      if ( CurrentFunc != PrevFunc ) // Function was changed
      {
        Event = EV_FUNC_FIRST;       // Generate FUNC_FIRST event
        PrevFunc = CurrentFunc;      // Save the function
        if ( EvCounter )             /// Some keys are stil pressed
          EvCounter = 0xFF;            // Lock any key events until all keys are not realized
      }
      else
      {
        Event = EventQueue;          // Read event thar was generated in interrupt handlers
        EventQueue = 0;              // The interrupt handlers can write new value
      }

      FuncArray[CurrentFunc]();      // Run the current menu function
      RedrawFlag = 0;
    }
#if defined(DISPLAY_N3310)
    else
    {
      if(RedrawFlag == 0)
      {
        RedrawFlag = 1;
        if(Display[0] != DisplayOld[0])
        {
          Draw(0);
          DisplayOld[0] = Display[0];
        }
        if(Display[1] != DisplayOld[1])
        {
          Draw(1);
          DisplayOld[1] = Display[1];
        }
        if(Display[2] != DisplayOld[2])
        {
          Draw(2);
          DisplayOld[2] = Display[2];
        }
      }
    }
#endif
 }
} // end main


uint8_t Event;     // Generated event
uint8_t EvCounter; // Press delay counter

uint8_t CurrentFunc = 0; // Current function

uint8_t MenuCounter; // Additional variable

uint8_t EventQueue; // Generated event
#define KEY_PRESSED_VALUE 2 // Counter for key press event
#define KEY_LONG_VALUE 70  // Counter for long press event
#define KEY_REPEATE_VALUE 80 // Counter for repeate event
static uint8_t PrevKey; // Previous keys pressed
#define REALIZE_KEY_VALUE 5 // Counter for realize event
static uint8_t RealizeCounter; // Realize delay counter


static  void check_keys(void)
{
  uint8_t Key;
  
  if ( EventQueue != 0 ) /* Previous event hasn't been handled */
    return;
  if (ResetCounter != 0)
  {
    ResetCounter--;
    if (ResetCounter == 1)
    {
      CurrentFunc = MAIN_FUNCTION;
    }
  }

  Key = (~KEYPORT) & KEY_MASK; /* Read the port */

  if ( Key == 0 ) // no any key pressed
  {
    if ( PrevKey == 0 ) // no any key saw pressed before
      return;
    if ( EvCounter > KEY_PRESSED_VALUE )
    {
      RealizeCounter++; // increase timer counter
      if ( RealizeCounter > REALIZE_KEY_VALUE ) // expired realise timeout
      {
        if ( EvCounter != 0xFF ) /* There is no switch to new function. New function should not get previos function event */
        {
          EventQueue = EV_KEY_REALIZED | PrevKey; /* Realized event - the last event */
        }
        EvCounter = 0; // Reset interval timer value
        PrevKey = 0;   // Reset key pressed value
        RealizeCounter = 0;  // Reset realise counter
      }
    }
    else
    {
      EvCounter = 0; // Reset interval timer value
      PrevKey = 0;   // Reset key pressed value
      RealizeCounter = 0;  // Reset realise counter
    }
  }
  else // Some keys are pressed
  {
    RealizeCounter = 0; //reset realise delay
   
    if ( EvCounter == 0xFF ) /* Locked - new function has been set */
      return; 
    
    if ( Key & (~PrevKey) ) //there are some new keys
    {
      PrevKey |= Key;       // adding the new keys
      if ( EvCounter == 0 )
      {
      /* Generate KEY TOUCH event */
        EventQueue = EV_KEY_TOUCH + PrevKey;
        ResetCounter = 3000;
      }
      else if ( EvCounter > KEY_LONG_VALUE ) // Delay after first press is not long
        EventQueue = EV_KEY_LONG | PrevKey; //generate key press event
    }
    else // the same keys are pressed
    {
      if ( EvCounter == KEY_PRESSED_VALUE ) // Delay after first press is not long
        EventQueue = EV_KEY_PRESSED | PrevKey; //generate key press event
      else if ( EvCounter == KEY_LONG_VALUE )  // Long press timeout has expired
      {
        EventQueue = EV_KEY_LONG | PrevKey; // Generate Long press event
      }
      else if ( EvCounter == KEY_REPEATE_VALUE ) // After long press the key is stil pressed
      {
        EventQueue = EV_KEY_REPEATE | PrevKey; // Generate repeate press event
        EvCounter = KEY_LONG_VALUE; // Reset time counter for next delay
      }
      EvCounter++; // Delay counter increasing
    }
  }
}

int8_t Counter595;

ISR (ADC_vect)
{
  int AdcValue; // Измеренное значение
  unsigned int Wv;//Взвешенное по 8 точкам значение

  AdcValue = ADCW; //сохраняем результат измерения

  Summ = Summ + ((long)AdcValue)*AdcValue;//Вычисляем сумму квадратов
  LastValues[CurrentLast] = AdcValue; //сохраняем последнее значение
  LastValues[CurrentLast+8] = AdcValue; //дважды что бы взвешенную сумму считать линейно
//  ADCSRA|=0x40; // Нафига снова запускать?

  if ( Counts > 130 ) // прошло больше чем половина второго полпериода (96)
  {
     //расчет взвешенного значения
     Wv = (LastValues[CurrentLast+1] + LastValues[CurrentLast + 8]) * 1 +
       (LastValues[CurrentLast+2] + LastValues[CurrentLast + 7]) * 2 +
       (LastValues[CurrentLast+3] + LastValues[CurrentLast + 6]) * 3 +
       (LastValues[CurrentLast+4] + LastValues[CurrentLast + 5]) * 4; 
		  
     if ( MaxWv <= Wv && Counts < 900 ) // мы еще не перевалили максимум в полупериоде
      {
         MaxWv = Wv; 		// сохранить новое максимальное значение
         MinWv = 0xFFFF;	// установить начальное минимальное значение
      }
      else // перевалили через максимум
      {
         if ( Wv < MinWv && Counts < 900 ) // если не перевалили через минимум
         MinWv = Wv; // установить новое минимальное значение
         else //Перевалили через минимум - конец серии измерений на полупериоде
         {
            LastSumm = Summ; // сохранение суммы квадратов
            LastCounts = Counts; // установка флага окончания серии измерений
            Summ = 0; // все
            Counts = -1; // переменные
            MaxWv = 0; // в исходное
            MinWv = 0xFFFF; // состояние
         }
      }
   }

  Counts++; // увеличить счетчик измерений
  // передвинуть место сохранения измерения
  CurrentLast = (CurrentLast + 1) & 0x7; /* from 0 to 7 */

#if !defined(DISPLAY_N3310)
  //Обновление индикатора
  SHIFT595PORT &= ~SHIFT595;
  if (Counter595 >= 0 )
  {
    uint8_t BitsToShift = Display[CurrentDigit];
#if !defined(DISPLAY_OA)
    if ( BitsToShift & (1<<Counter595))
#else
    if ( !(BitsToShift & (1<<Counter595)))
#endif
    {
      DATA595PORT |= DATA595;
    }
    else
    {
      DATA595PORT &= ~DATA595;
    }
    Counter595--;
    SHIFT595PORT |= SHIFT595; // shift the data into serial register 595
  }
  else
  {
    STORE595PORT &= ~STORE595;
#if !defined(DISPLAY_OA)
    DIGPORT |= (DIG1|DIG2|DIG3);//отключили все общие - катоды
#else
    DIGPORT &= ~(DIG1|DIG2|DIG3);//отключили все общие - аноды
#endif
    Counter595 = 7;
    STORE595PORT |= STORE595;

    CurrentDigit++; // Следующий раз отобразить следующую цифру
    if ( CurrentDigit >= 3 ) // и так по кругу
      CurrentDigit = 0;
#if !defined(DISPLAY_OA)
    DIGPORT &= DigitLine[CurrentDigit]; //подключили соотв общий катод
#else
    DIGPORT |= DigitLine[CurrentDigit]; //подключили соотв общий катод
#endif
  }
#endif /* DISPLAY_N3310 */

}
