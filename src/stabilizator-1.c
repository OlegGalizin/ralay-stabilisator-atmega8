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

#include "menu.h"

#define F_CPU 8000000UL  

#define Coeff 16  //Коэффициент пропорциональности напряжения
#define RmsP2(V) ((unsigned int)(((double)V)*V/Coeff/Coeff)) // макрос перевода вольт в попугаи

#define PinOn _BV(PC5)   // пин реле включения нагрузки
#define Pin_235 _BV(PD0) // пин реле если сеть больше 238В
#define Pin_200 _BV(PD1) // пин реле если сеть меньше 200В
#define Pin_180 _BV(PD3) // пин реле если сеть меньше 180В

static  void check_keys(void);


unsigned char CurrentDigit; // Текущая отображаемая цифра
volatile unsigned char Display[3]; //отображаемые цифры ( диапазон 0 - 9)

char CurrentLast = 0;//Где в массиве лежит последнее значение
char Diapazon = 0; 	//текущий диапазон 0-выкл, 1=+20%, 2=+10%, 3=+0%, 4=-10%

EEMEM uint16_t ee_U_max = 245;		//максимальное наряжение сети
EEMEM uint16_t ee_U_min = 160;		//минимальное напряжение сети
EEMEM uint16_t ee_zadergka = 100;	//времья задержки на включение
EEMEM uint16_t ee_filter = 50;		//времья нечувствительности колебаний сетевого напряжения

volatile uint16_t U_max;// = 245;  //максимальное наряжение сети
volatile uint16_t U_min;// = 160; //минимальное напряжение сети
volatile uint16_t U_maxUE;// = 245;  //максимальное наряжение сети
volatile uint16_t U_minUE;// = 160; //минимальное напряжение сети
volatile uint16_t zadergka; // Задержка включения нагрузки после аварии
volatile uint16_t t_filter; // времья нечуствительности колебаний сетевого напряжения

volatile int Timer = 0; // Таймер задержки включения: 50 периодов  - 1 сек , 250 - 5 сек
volatile int tm1 = 0;
volatile char key_code = 0;

#define SEG_A 0x01
#define SEG_B 0x02
#define SEG_C 0x04
#define SEG_D 0x08
#define SEG_E 0x10
#define SEG_F 0x20
#define SEG_G 0x40
#define SEG_H 0x80
#define SEG_DOT SEG_H

// Общий катод,  размещение  сегментов в разрядах  -  hgfedcba
const unsigned char digit[11]=
{0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x00,}; // 0 1 2 3 4 5 6 7 8 9 откл.  
const unsigned char DigitLine[3] = {0x4,0x8,0x10}; //Какую цифру отображать

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
  unsigned char* Pointer = Display;
  char i = 3;
  char OnFlag = 0;

  while (i)
  {
     char Dig = DataForLed/Div;

     if ( Dig == 0 && OnFlag == 0)
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

static unsigned char RelayCounter;

#define CheckRelay() RelayCounter++; \
  if ( RelayCounter < t_filter ) \
    return;
#define ClearRelay() RelayCounter = 0;

static void Regulator (void)
{
  if ( Timer < zadergka ) // 5 сек. задержка: 50 периодов=1 сек, 250 = 5 сек
  {
    Timer = Timer + 1;
  }
  else
  {   
    switch (Diapazon)
    {
      case 1: //Входное 228 - U_max вольт
        if ( ImmediateValue > (U_maxUE) )
        {
//         CheckRelay();
         goto D0;
        }
        if ( ImmediateValue < RmsP2(228) )
        {
         CheckRelay();
         goto D2;
        }
        ClearRelay();
        return;
      case 2: //Входное 200 - 235 вольт
        if ( ImmediateValue > RmsP2(235) )
        {
         CheckRelay();
         goto D1;
        }
        if ( ImmediateValue < RmsP2(200) )
        {
         CheckRelay();
         goto D3;
        }
        ClearRelay();
        return;
      case 3: //Входное 180 - 210 вольт
        if ( ImmediateValue > RmsP2(210) )
        {
         CheckRelay();
         goto D2;
        }
        if ( ImmediateValue < RmsP2(180) )
        {
         CheckRelay();
         goto D4;
        }
        ClearRelay();
        return;
      case 4: //Входное U_min - 190 вольт
        if ( ImmediateValue > RmsP2(190) )
        {
         CheckRelay();
         goto D3;
        }
        if ( ImmediateValue < (U_minUE) )
        {
         CheckRelay();
         goto D0;
        }
        ClearRelay();
        return;
		  
      default:
        if (ImmediateValue > RmsP2(235) && (ImmediateValue < (U_maxUE-10))) // U_max-10
         goto D1;
        if (ImmediateValue > RmsP2(200) && (ImmediateValue < RmsP2(235)))
         goto D2;
        if (ImmediateValue < RmsP2(200) && (ImmediateValue > RmsP2(180)))
         goto D3;
        if (ImmediateValue < RmsP2(180) && (ImmediateValue > (U_minUE+10))) // U_min+10
         goto D4;
        Timer = 0;         // Сброс таймера задержки включения
      } // switch
    } // if
    
    return;
	
D1: // Включение 1 диапазона 228 - U_max
  PORTD |= Pin_235;  // включить реле   -20В
  PORTD &= ~Pin_200; // отключить реле   +20В
  PORTD &= ~Pin_180; // отключить реле   +40В
  Diapazon = 1;
  PORTC |= PinOn;    // Включить выход
  goto DisplayVoltage;
D2: // Включение 2 диапазона 200 - 235
  PORTD &= ~Pin_235; // отключить реле   -20В
  PORTD &= ~Pin_200; // отключить реле   +20В
  PORTD &= ~Pin_180; // отключить реле   +40В
  Diapazon = 2;
  PORTC |= PinOn;    // Включить выход
  goto DisplayVoltage;
D3: // Включение 3 диапазона 180 - 210
  PORTD &= ~Pin_235;  // отключить реле   -20В
  PORTD |= Pin_200;   // включить реле   +20В
  PORTD &= ~Pin_180;  // отключить реле   +40В
  Diapazon = 3;
  PORTC |= PinOn;    // Включить выход
  goto DisplayVoltage;
D4: // Включение 4 диапазона U_min - 190
  PORTD &= ~Pin_235;  // отключить реле   -20В
  PORTD |= Pin_200; 	 // включить реле   +20В
  PORTD |= Pin_180; 	 // включить реле   +40В
  Diapazon = 4;
  PORTC |= PinOn;    // Включить выход
  goto DisplayVoltage;
D0: // Включение 0 диапазона - все выключено
  Timer = 0;         // Сброс таймера задержки включения
  Diapazon = 0;
  PORTC &= ~PinOn;   // ОТКЛЮЧИТЬ ВСЕ РЕЛЕ И НАГРУЗКУ!!!
  PORTD &= ~Pin_235; // отключить реле   -20В
  PORTD &= ~Pin_200; // отключить реле   +20В
  PORTD &= ~Pin_180; // отключить реле   +40В

DisplayVoltage:
  {
    uint16_t Out = (uint16_t)(sqrt(ImmediateValue)*Coeff); //Out moving average value

    Decoder_display(Out);
    Display[2] |= SEG_DOT;
    DisplayCounter = -100;
  }
}


static void Init (void)
{
// Input/Output Ports initialization
// Port B initialization
	PORTB=0x00;
	DDRB=0xFF;

// Port C initialization
	PORTC=0x00; // PORTC=0x03 поддтяжка включенна!!! для АЦП лучше выключить, т.е. PORTC=0x00
	DDRC=0x3C; // 0011 1100

// Port D initialization
	PORTD=0xE4;
	DDRD=0x1B; // 0001 1011    PinOn | Pin_235 | Pin_200 | Pin_180;

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
	ADMUX=_BV(REFS1)|_BV(REFS0); /* 2.56v reference & ADC0 */
	ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADIE)|_BV(ADPS1)|_BV(ADPS2);  // div64
// ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADIE)|_BV(ADPS0)|_BV(ADPS2);  // div32

	U_max = eeprom_read_word( &ee_U_max);
        if ( U_max == 0xFFFF )
        {
          U_max = 245;
        }
        U_maxUE = RmsP2(U_max);

	U_min = eeprom_read_word( &ee_U_min);
        if ( U_min == 0xFFFF )
        {
          U_min = 160;
        }
        U_minUE = RmsP2(U_min);
		  
	zadergka = eeprom_read_word( &ee_zadergka);
        if ( zadergka == 0xFFFF )
        {
          zadergka = 100;
        }
	
	t_filter = eeprom_read_word( &ee_filter);
        if ( t_filter == 0xFFFF )
        {
          t_filter = 50;
        }
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

uint16_t Value;

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
      if ( Value < 179 )
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
    if ( (Event & KEY_MASK) == KEY_ENTER )
    {
      CurrentFunc = HERTZ_FUNCTION;
      return;
    }
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
        if ( MenuCounter < 4 )
          MenuCounter++;
        else
          MenuCounter = 0;
        break;
      case KEY_DOWN:
        if ( MenuCounter > 0 )
          MenuCounter--;
        else
          MenuCounter = 4;
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
     Display[0] = SEG_E|SEG_G; // e
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
  }
}


const MenuFunction_t FuncArray[] = {MainFunction, 
  HertzFunction,MenuFunction, HiFunction, LoFunction,
  DelayFunction,FilterFunction,SavedFunction}; // Array of functions of the menu



//*********************************************************


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
    }
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



ISR (ADC_vect)
{
  int AdcValue; // Измеренное значение
  unsigned int Wv;//Взвешенное по 8 точкам значение

  AdcValue = ADCW; //сохраняем результат измерения

  Summ = Summ + ((long)AdcValue)*AdcValue;//Вычисляем сумму квадратов
  LastValues[CurrentLast] = AdcValue; //сохраняем последнее значение
  LastValues[CurrentLast+8] = AdcValue; //дважды
  ADCSRA|=0x40;

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

  //Обновление индикатора
  PORTB = 0; //отключили все сегменты - аноды
  PORTC &= 0xE3;//отключили все сегменты общие - катоды
 
  PORTB = Display[CurrentDigit]; // установили в 1 сегменты соотв цифре
  PORTC |= DigitLine[CurrentDigit]; //подключили соотв общий катод
  CurrentDigit++; // Следующий раз отобразить следующую цифру
  if ( CurrentDigit >= 3 ) // и так по кругу
    CurrentDigit = 0;
}
