// mdjoy.ino
// Author: mistepien@wp.pl
// Copyright 2023
//
// Based on https://github.com/jonthysell/SegaController
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*----------------------------------------
  |   DDRx   |   PORTx  |    result      |
  ---------------------------------------|
  ---------------------------------------|
  |    0     |     0    |     INPUT      |
  ----------------------------------------
  |    0     |     1    |  INPUT_PULLUP  |
  ----------------------------------------
  |    1     |     0    | OUTPUT (LOW)   |
  ----------------------------------------
  |    1     |     1    | OUTPUT (HIGH)  |
  ----------------------------------------

  LOW(0) state of both registers is DEFAULT,
  thus every pin is in INPUT mode without doing anything.

  ------------------            -----------------
  |  HARDWARE XOR  |            |  SOFTWARE XOR |
  ------------------            -----------------
  PINx = byte;    <========>    PORTx ^= byte;

  ------------------
  |  HARDWARE XOR  |
  ------------------

  "The port input pins I/O location is read only, while the data register and the
  data direction register are read/write. However, writing a logic one to a bit
  in the PINx register, will result in a toggle in the
  corresponding bit in the data register."

  That is more efficient since XOR operation is done in hardware, not software,
  and it saves cycles since in code there is no need to bother about XOR.*/

/*Source of these classic rapidfire frequencies - great input of Nightshft
  (Nightshft, thank you very much!):
  https://eab.abime.net/showpost.php?p=1364366&postcount=3

FREQ0 = 7 CPS  (Quickgun Turbo Pro)
FREQ1 = 13 CPS (Zipstick)
FREQ2 = 29 CPS (Quickshot 128F)
FREQ3 = 62 CPS (Competition Pro - Transparent model / Quickshot TopStar SV 127)

*/

#define __7X_VER__

#if !((F_CPU == 1000000L) || (F_CPU == 2000000L))
#error This is designed only for 1MHz and 2MHz clock!!!
#endif


#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#endif

#if defined(__AVR_ATmega48PB__) || defined(__AVR_ATmega88PB__) || defined(__AVR_ATmega168PB__) || defined(__AVR_ATmega328PB__)
#define DDR_REG_PULLUP DDRE
#define PORT_REG_PULLUP PORTE
#define PIN_REG_PULLUP PINE
#elif defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
#define DDR_REG_PULLUP DDRA
#define PORT_REG_PULLUP PORTA
#define PIN_REG_PULLUP PINA
#endif


//OUTPUT PINS FOR JOYSTICK
enum joystick_pinout {
  LBTN = 0,     //PB0
  DBTN = 1,     //PB1
  UBTN = 2,     //PB2
  F1BTN = 3,    //PB3
  F3BTN = 4,    //PB4
  F2BTN = 5,    //PB5
  C64MODE = 6,  //PB6
  RBTN = 7      //PB7
};


//UBTN/DBTN/LBTN/RBTN/F1BTN are in reversed logic (diodes between MCU and DB9)
//they are "NOT PRESSED" when they are HIGH
constexpr byte ATARI_JOY = bit(UBTN) | bit(DBTN) | bit(LBTN) | bit(RBTN) | bit(F1BTN);



//OUTPUT PINS FOR LEDS
#if defined(__7X_VER__)
enum leds_pullout_pinout {
  F3PULLUP = 0,     //PC0
  F2PULLUP = 1,     //PC1
  AUTOFIRELED = 2,  //PC2
  CONF0LED = 3,     //PC3
  CONF1LED = 4,     //PC4
  MODELED = 5       //PC5
};

#else
enum leds_pullout_pinout {
  CONF0LED = 0,     //PC0
  CONF1LED = 1,     //PC1
  MODELED = 2,      //PC2
  AUTOFIRELED = 3,  //PC3
  F3PULLUP = 4,     //PC4
  F2PULLUP = 5      //PC5
};
#endif

constexpr byte ALL_LEDS = bit(CONF0LED) | bit(CONF1LED) | bit(AUTOFIRELED) | bit(MODELED);

#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>
//#include <util/atomic.h>
#include <EEPROM.h>
#include "SegaController.h"
#include "Timer_params.h"

constexpr word _c64_amiga_combination = bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_DPAD_DOWN) | bit(SC_DPAD_RIGHT);
constexpr word _pullup_combination = bit(SC_BTN_A) | bit(SC_BTN_C) | bit(SC_DPAD_UP) | bit(SC_DPAD_LEFT);
constexpr word _gamepad_possible_buttons = 0xFFFF & ~(bit(SC_MODE));

bool DPAD_UP = 0;
bool DPAD_DOWN = 0;
bool DPAD_LEFT = 0;
bool DPAD_RIGHT = 0;
bool BTN_UP = 0;
bool BTN_DOWN = 0;

word prev_gamepad_state;
byte rapidfire_freq = 0;
bool timer_start_flag = 0;     //it will let to start timer together with update of PORT registers
volatile bool fire_rapid = 0;  //this variable is toggled by timer interrupt (0101010101)
volatile bool reading_controller_flag = 0;
volatile bool blinking_led = 0;
volatile bool save_eeprom_flag = 0;
bool ctl_on_flag = 0;
bool rapidfire_button = 0;     //state of rapid button
bool autofire_button = 0;      //"artificial" state of autofire_button
bool rapidfire_sw = 0;         //to jest "stan" przełączenia przycisku FIRE(zwykłego) w tryb rapidfire
bool rapidfire_sw_button = 0;  //to jest wciśnięcie przycisku FIRE będącego w trybie rapidfire
bool fire_single = 0;          //to jest zwykły fire wciśnięty przez usera -- wciśnięty to strzał i tyle
bool rapid_up_down_btn = 0;
bool rapid_left_right_btn = 0;
bool start_btn = 0;
bool reset_flag = 0;
byte ledstate = 0;
byte joystate = 0;
constexpr byte max_blinks = 10;

typedef struct {
  byte _counter;
  byte _packed_data;
} eeprom_buffer_struct;


/*
_packed_data byte:
5 - AMIGAmode_bit
2 - pullup_mode_bit
3, 4 / 0, 1 - joyconf //it follows PCB design and PC port
7, 6 - rapidefire_freq
*/

constexpr byte rapidfire_hbit = 7;
constexpr byte rapidfire_lbit = 6;
constexpr byte AMIGAmode_bit = 5;
constexpr byte pullup_mode_bit = 2;

eeprom_buffer_struct eeprom_stuff;
byte eeprom_stuff_index;
constexpr byte sizeof_eeprom_stuff = sizeof(eeprom_stuff);
constexpr byte eeprom_stuff_last_before_max_index = ((255 / sizeof_eeprom_stuff) * sizeof_eeprom_stuff) - sizeof_eeprom_stuff;

void setup() {
  DDRB = 0xFF;  //OUTPUT: PB0-7:
  //DDRB |= ATARI_JOY | bit(F2BTN) | bit(F3BTN) | bit(C64MODE);

  /*
  UP,DOWN,RIGHT,LEFT and F1BTN are in reversed logic -
  "not pressed" ("released") is 1, so OUTPUT is set to HIGH
  F2BTN and F3BTN are set to 0 */
  PINB = ATARI_JOY;  //PORTB is default to LOW, toggle to HIGH only ATARI_JOY

  //pull-up stuff
  DDRC |= ALL_LEDS | bit(F3PULLUP) | bit(F2PULLUP);  //set all (except UNUSED_LP)to output
                                                     //low by default

  wdt_disable();

  //INITIALIZE SEGA PAD
  sega.begin(4, 0, 1, 2, 3, 6, 5);

  //deal with unused pins
  PORTD |= bit(PD7);  //set to INPUT_PULLUP

#if defined(__AVR_ATmega48PB__) || defined(__AVR_ATmega88PB__) || (__AVR_ATmega168PB__) || defined(__AVR_ATmega328PB__)
  byte _PE_Pins = bit(PE0) | bit(PE1) | bit(PE2) | bit(PE3);
  //DDRE &=  ~_PE_Pins;
  PORTE |= _PE_Pins;
#elif defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
  byte _PA_Pins = bit(PA0) | bit(PA1) | bit(PA2) | bit(PA3);
  //DDRA &=  ~_PA_Pins;
  PORTA |= _PA_Pins;

  PORTC |= bit(PC7);  //set to INPUT_PULLUP
#endif

  eeprom_buffer_struct tmp_eeprom_stuff = read_eeprom_stuff_packed_data();

  /*#############################################
  C64/AMIGA MODE stuff
  AMIGAmode = global variable -- is a logic negation of SW1 NET on PCB
  AMIGAmode = 0 -- C64 mode -- so C64 mode does not draw power from pull_down resitor
  AMIGAmode = 1 -- AMIGA mode */

  set_C64_AMIGA_MODE_in_setup(bitRead(tmp_eeprom_stuff._packed_data, AMIGAmode_bit));

  timer_instead_of_millis();

  byte _ledstate_setup = 0;
  bool __blinking_led__ = 0;
  byte reads_num = 0;
  word _sega_state_in_setup = 0;
  byte __mode_combination_pressed__ = 0;

  while (reads_num < max_blinks) {

    if (__blinking_led__) {
      _ledstate_setup = tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(MODELED) | bit(AUTOFIRELED);
    } else {
      _ledstate_setup = 0;
    }

    push_ledstate_to_register(_ledstate_setup);

    if (reading_controller_flag) {
      _sega_state_in_setup = sega.getState();
      if (is_C64_AMIGA_mode_combination_pressed_in_setup(_sega_state_in_setup)) {
        bitSet(__mode_combination_pressed__, AMIGAmode_bit);
      } else if (is_pullup_mode_combination_pressed_in_setup(_sega_state_in_setup)) {
        bitSet(__mode_combination_pressed__, pullup_mode_bit);
      }
      reading_controller_flag = 0;
    }

    if (__mode_combination_pressed__) {
      break;
    }

    __blinking_led__ = blinking_led;
    reads_num += catcher_timer_flag(__blinking_led__);
  }

  tmp_eeprom_stuff._packed_data ^= __mode_combination_pressed__;

  if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
    set_C64_AMIGA_MODE_in_setup(bitRead(tmp_eeprom_stuff._packed_data, AMIGAmode_bit));
  }

  set_pullup_mode_in_setup(tmp_eeprom_stuff._packed_data);

  eeprom_stuff = tmp_eeprom_stuff;

  if (__mode_combination_pressed__) {
    smart_eeprom_stuff_put();
    if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
      blinking_leds_after_combination(ALL_LEDS);
    } else if (__mode_combination_pressed__ & bit(pullup_mode_bit)) {
      blinking_leds_after_combination(tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(MODELED) | bit(AUTOFIRELED));
    }
  }


  //set joyconf
  byte tmp_joyconf = (tmp_eeprom_stuff._packed_data & (bit(CONF0LED) | bit(CONF1LED))) >> CONF0LED;
  set_joyconf(tmp_joyconf);

  //set rapidfire frequency
  rapidfire_freq = (tmp_eeprom_stuff._packed_data & (bit(rapidfire_hbit) | bit(rapidfire_lbit))) >> rapidfire_lbit;

  //power stuff -- power only necessary things
  power_all_disable();
#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
  power_timer0_enable();  //millis()
#endif
  power_timer1_enable();  //autofire
#if !defined(__AVR_ATtiny88__) && !defined(__AVR_ATtiny48__)
  power_timer2_enable();  //reading controller
#endif

  timer_stop();

  wdt_enable(WDTO_15MS);
}

void set_C64_AMIGA_MODE_in_setup(bool ___AMIGAmode___) {
  /*if (___AMIGAmode___) {  //C64MODE pin is set to LOW, AMIGAmode flag is 1  -- that is AMIGA mode
    bitClear(PORTB, C64MODE);
  } else {
    bitSet(PORTB, C64MODE);  //C64MODE pin is set to HIGH, AMIGAmode flag is 0 -- that is C64 mode
  }*/
  bitWrite(PORTB, C64MODE, 1 ^ ___AMIGAmode___);
}

void set_pullup_mode_in_setup(byte ___AMIGAmode___pullup_mode___) {
  if (sega.complex_bool_value(___AMIGAmode___pullup_mode___, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
    update_pull_up_register_in_AMIGA_mode(bit(F2BTN) | bit(F3BTN));
  } /* PULLUP is now LOW and F2BTB/F3BTN are LOW
         we set PULLUP to HIGH and F2BTB/F3BTN
         since now we can XOR PULLUP and F2BTB/F3BTN
         with same data and they will be "xored" to each
         other */
}

bool is_C64_AMIGA_mode_combination_pressed_in_setup(word _sega_state_) {
  bool _comb_pressed = (_sega_state_ & _gamepad_possible_buttons) ^ (_c64_amiga_combination | bit(SC_CTL_ON)) ? 0 : 1;  //xor = 1 -- combination is NOT PRESSED
  return _comb_pressed;
}

bool is_pullup_mode_combination_pressed_in_setup(word _sega_state_) {
  bool _comb_pressed = (_sega_state_ & _gamepad_possible_buttons) ^ (_pullup_combination | bit(SC_CTL_ON)) ? 0 : 1;  //xor = 1 -- combination is NOT PRESSED
  return _comb_pressed;
}


byte catcher_timer_flag(bool __flag_to_catch) {
  static bool __prev_flag_to_catch;
  if (__flag_to_catch ^ __prev_flag_to_catch) {
    __prev_flag_to_catch = __flag_to_catch;
    return 1;
  } else {
    return 0;
  }
}

void blinking_leds_after_combination(byte __leds__) {
  byte reads_num = 0;
  bool __blinking_led__ = 0;
  byte _ledstate_setup = 0;
  while (reads_num < max_blinks) {
    __blinking_led__ = blinking_led;
    _ledstate_setup = __blinking_led__ ? __leds__ : 0;
    push_ledstate_to_register(_ledstate_setup);
    reads_num += catcher_timer_flag(__blinking_led__);
  }
}


#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
void timer_instead_of_millis() {
  noInterrupts();
  // Clear registers
  TCNT0 = 0;


#if (F_CPU == 1000000L)
  // 269.3965517241379 Hz (1000000/((57+1)*64))
  OCR0A = 57;
#elif (F_CPU == 2000000L)
  // 269.3965517241379 Hz (2000000/((115+1)*64))
  OCR0A = 115;
#endif

  // CTC && Prescaler 64
  TCCR0A = bit(CTC0) | bit(CS01) | bit(CS00);

  // Output Compare Match A Interrupt Enable
  TIMSK0 = bit(OCIE0A);
  interrupts();
}
#else
void timer_instead_of_millis() {
  noInterrupts();
  // set CTC & clear all other bits in TCCR2A
  TCCR2A = bit(WGM21);

  TCNT2 = 0;

#if (F_CPU == 1000000L)
  // 271.7391304347826 Hz (1000000/((114+1)*32))
  OCR2A = 114;
#elif (F_CPU == 2000000L)
  // 270.56277056277054 Hz (2000000/((230+1)*32))
  OCR2A = 229;
#endif

  // Prescaler 32
  TCCR2B = bit(CS21) | bit(CS20);

  // Output Compare Match A Interrupt Enable
  TIMSK2 = bit(OCIE2A);
  interrupts();
}
#endif

#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
ISR(TIMER0_COMPA_vect) {
#else
ISR(TIMER2_COMPA_vect) {
#endif
  reading_controller_flag = 1;

  static unsigned int _counter_eeprom;
  if (_counter_eeprom == 0) {
    save_eeprom_flag = 1;
  }
  if (_counter_eeprom++ > 1200) {  //1200 * 3,7ms = 4440 ms =4.44s
    _counter_eeprom = 0;
  }

  static byte _counter;
  if (rapidfire_sw | (1 ^ ctl_on_flag)) {
    if (_counter++ > 40) {  //40 * 3,7ms = 148ms
      _counter = 0;
      blinking_led ^= 1;
    }
  } else {
    _counter = 0;
    blinking_led = 0;
  }
}


void timer_start(byte rspeed = 0) {
  noInterrupts();

  // set CTC & clear all other bits in TCCR1B
  TCCR1B = bit(WGM12);

  OCR1A = timer_params[rspeed].ocr1a;

  // set prescaler
  TCCR1B |= timer_params[rspeed].prescaler ? bit(CS11) : bit(CS10);

  /*----------------------------------------------------------------------------
    look a file Timer_params.h -- what Prescalers are really used in Timer_params.h?
    ---------------------------------------------------------------------------*/
  // set prescaler
  /* switch (timer_params[rspeed].prescaler) {
    case 0:  //1
      TCCR1B |= bit(CS10);
      break;
    case 1:  //8
      TCCR1B |= bit(CS11);
      break;
    case 2:  //64
      TCCR1B |= bit(CS11) | bit(CS10);
      break;
    case 3:  //256
      TCCR1B |= bit(CS12);
      break;
    case 4:  //1024
      TCCR1B |= bit(CS12) | bit(CS10);
      break;
  } */

  // Output Compare Match A Interrupt Enable
#if defined(__AVR_ATmega8__)
  TIMSK |= bit(OCIE1A);
#else
  TIMSK1 |= bit(OCIE1A);
#endif

  interrupts();
}

void timer_stop() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

#if defined(__AVR_ATmega8__)
  TIMSK = 0;
#else
  TIMSK1 = 0;
#endif

  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  fire_rapid ^= 1;
}

void rapidfire_toggle(bool state) {
  static byte rapidfire_freq_prev;
  static bool rapidfire_toggle_state_prev;
  bool changed_rapidfire_toggle_state = state ^ rapidfire_toggle_state_prev;
  byte changed_rapidfire_freq = rapidfire_freq ^ rapidfire_freq_prev;

  if (changed_rapidfire_toggle_state || changed_rapidfire_freq) {
    if (changed_rapidfire_toggle_state) {
      rapidfire_toggle_state_prev = state;
    }
    if (changed_rapidfire_freq) {
      rapidfire_freq_prev = rapidfire_freq;
    }

    timer_stop();

    if (state) {
      timer_start_flag = 1;  //do not start timer now but just before pushing joystate to PORTB
    }
    fire_rapid = state;
  }
}

void set_rapidfire_freq(byte freq) {
  rapidfire_freq = freq & 3;
}

void set_joyconf(byte conf) {
  static byte _prev_conf;
  conf &= 3;
  if (_prev_conf ^ conf) {
    ledstate &= ~(bit(CONF0LED) | bit(CONF1LED));  //clear joyconf bits
    ledstate |= (conf << CONF0LED);                //set joyconf bits
    _prev_conf = conf;
    reset_flag = 1;
  }
}

void reset_buttons() {
  fire_single = 0;
  rapidfire_button = 0;
  rapid_up_down_btn = 0;
  rapid_left_right_btn = 0;
  autofire_button = 0;
  start_btn = 0;
  bitClear(joystate, F2BTN);
  bitClear(joystate, F3BTN);
  BTN_UP = 0;
  BTN_DOWN = 0;
  prev_gamepad_state &= (bit(SC_CTL_ON) | bit(SC_MODE) | bit(SC_DPAD_UP) | bit(SC_DPAD_DOWN) | bit(SC_DPAD_LEFT) | bit(SC_DPAD_RIGHT) | bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_BTN_HOME));
  //prev_gamepad_state &= (bit(SC_CTL_ON) | bit(SC_MODE) | bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_BTN_HOME));
}

void button(byte btn, bool btn_state) {  //that function is used in loop reading every bit of controller's state
  //and here we make a "choice" between different configurations
  button_basic(btn, btn_state);

  if (start_btn) {
    button_pressed_WITH_start_btn(btn, btn_state);  //   stb = 1, 6btn=x, alt=x
  } else {
    button_pressed_WITHOUT_start_btn(btn, btn_state);  //   stb = 0, 6btn=x, alt=x

    /*
    enum leds_pullout_pinout -- 7X_VER
    CONF0LED = 3,     //PC3
    CONF1LED = 4,     //PC4
    MODELED = 5,      //PC5

    enum leds_pullout_pinout -- 8X_VER
    CONF0LED = 0,     //PC0
    CONF1LED = 1,     //PC1
    MODELED = 2,      //PC2
    */

    switch ((ledstate >> CONF0LED) & 7) {
      case 0:
        button_conf0_3btnmode(btn, btn_state);
        break;
      case 1:
        button_conf1_3btnmode(btn, btn_state);
        break;
      case 2:
        button_conf2_3btnmode(btn, btn_state);
        break;
      case 3:
        button_conf3_3btnmode(btn, btn_state);
        break;
      case 4:
        button_conf0_6btnmode(btn, btn_state);
        break;
      case 5:
        button_conf1_6btnmode(btn, btn_state);
        break;
      case 6:
        button_conf2_6btnmode(btn, btn_state);
        break;
      case 7:
        button_conf3_6btnmode(btn, btn_state);
        break;
    }
  }
}

void btn_joyconf_toggle(bool btn_state) {
  if (btn_state) {
    set_joyconf(((ledstate & (bit(CONF0LED) | bit(CONF1LED))) >> CONF0LED) + 1);
  }
}

void set_btn(byte btn, bool btn_state) {
  bitWrite(joystate, btn, btn_state);
}

void button_basic(byte btn, bool btn_state) {
  switch (btn) {  //DPAD + CTL-ON + 6MODE + START_BTN
    case SC_CTL_ON:
      reset_flag = 1;
      ctl_on_flag = btn_state;
      break;
    case SC_MODE:
      reset_flag = 1;
      rapidfire_sw = 0;
      rapidfire_sw_button = 0;
      bitWrite(ledstate, MODELED, btn_state);
      break;
    case SC_BTN_START:  //START_BTN
      start_btn = btn_state;
      break;
    case SC_BTN_MODE:
      if (btn_state) {
        set_rapidfire_freq(rapidfire_freq + 1);
      }
      break;
    case SC_BTN_HOME:  //HOME BTN (8bitdo M30 2.4 controller)
      btn_joyconf_toggle(btn_state);
      break;
  }
}

void button_pressed_WITH_start_btn(byte btn, bool btn_state) {
  switch (btn) {
    case SC_DPAD_UP:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 0;
      } else {
        DPAD_UP = 0;
      }
      break;
    case SC_DPAD_DOWN:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 2;
      } else {
        DPAD_DOWN = 0;
      }
      break;
    case SC_DPAD_LEFT:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 3;
      } else {
        DPAD_LEFT = 0;
      }
      break;
    case SC_DPAD_RIGHT:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 1;
      } else {
        DPAD_RIGHT = 0;
      }
      break;
    case SC_BTN_A:  //+ START BTN
      if (btn_state) {
        btn_joyconf_toggle(btn_state);
      }
      break;
    case SC_BTN_B:                           //+ START BTN
      if (bitRead(ledstate, MODELED) ^ 1) {  //disable rapidfire_sw in 6-btn mode
        if (btn_state) {
          rapidfire_sw ^= 1;
          if (rapidfire_sw) {
            autofire_button = 0;
          } else {
            rapidfire_sw_button = 0;  //if rapidfire_sw=0 then rapidfire_sw_button=0, too
          }
        }
      } else {
        fire_single = btn_state;
      }
      break;
    case SC_BTN_C:  //+ START BTN
      if (btn_state) {
        set_rapidfire_freq(rapidfire_freq + 1);
      }
      break;
  }
}

void button_pressed_WITHOUT_start_btn(byte btn, bool btn_state) {
  switch (btn) {
    case SC_DPAD_UP:
      DPAD_UP = btn_state;
      break;
    case SC_DPAD_DOWN:
      DPAD_DOWN = btn_state;
      break;
    case SC_DPAD_LEFT:
      DPAD_LEFT = btn_state;
      break;
    case SC_DPAD_RIGHT:
      DPAD_RIGHT = btn_state;
      break;
    case SC_BTN_B:  //FIRE1
      if (rapidfire_sw) {
        rapidfire_sw_button = btn_state;
      } else {
        fire_single = btn_state;
      }
      break;
  }
}

void btn_rapidfire(bool btn_state) {
  rapidfire_button = btn_state;
}

void btn_autofire(bool btn_state) {
  if (btn_state) {
    autofire_button ^= 1;
    if (autofire_button) {
      rapidfire_sw = 0;
      if (rapidfire_sw_button) { /*fire button is pressed -- change its meaning
                                   from rapidfire_sw_button to file_single
                                  */
        rapidfire_sw_button = 0;
        fire_single = 1;
      }
    }
  }
}

void button_conf0_6btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_C:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SC_BTN_X:  //AUTOFIRE BUTTON
      btn_autofire(btn_state);
      break;
    case SC_BTN_Y:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SC_BTN_Z:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

void button_conf1_6btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SC_BTN_C:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_Z:  //AUTOFIRE BUTTON
      btn_autofire(btn_state);
      break;
    case SC_BTN_Y:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SC_BTN_X:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

void button_conf2_6btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_Z:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SC_BTN_C:  //DOWN BUTTON
      BTN_DOWN = btn_state;
      break;
    case SC_BTN_Y:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SC_BTN_X:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

void button_conf3_6btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_C:  //LEFT-RIGHT RAPID BUTTON
      rapid_left_right_btn = btn_state;
      break;
    case SC_BTN_X:  //F3
      set_btn(F3BTN, btn_state);
      break;
    case SC_BTN_Y:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SC_BTN_Z:  //UP-DOWN RAPID BUTTON
      rapid_up_down_btn = btn_state;
      break;
  }
}

void button_conf0_3btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SC_BTN_C:  //UP BUTTON
      BTN_UP = btn_state;
      break;
  }
}

void button_conf1_3btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_C:  //UP BUTTON
      BTN_UP = btn_state;
      break;
  }
}

void button_conf2_3btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //RAPIDFIRE
      btn_rapidfire(btn_state);
      break;
    case SC_BTN_C:  //F2
      set_btn(F2BTN, btn_state);
      break;
  }
}

void button_conf3_3btnmode(byte btn, bool btn_state) {
  switch (btn) {
    case SC_BTN_A:  //LEFT-RIGHT RAPID BUTTON
      rapid_left_right_btn = btn_state;
      break;
    case SC_BTN_C:  //UP-DOWN RAPID BUTTON
      rapid_up_down_btn = btn_state;
      break;
  }
}

void push_joystate_and_pullstate_to_register(byte _joystate) {
  //Pushing joystate to PORTB
  static byte prev_joystate;
  byte changed_joystate = prev_joystate ^ _joystate;
  if (changed_joystate) {
    PINB = changed_joystate;  //hardware XOR

    if (sega.complex_bool_value(eeprom_stuff._packed_data, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
      //if ((eeprom_stuff._packed_data & (bit(AMIGAmode_bit) | bit(pullup_mode_bit))) == (bit(AMIGAmode_bit) | bit(pullup_mode_bit))) { //ONLY in AMIGAmode
      byte changed_F2F3 = changed_joystate & (bit(F2BTN) | bit(F3BTN));
      update_pull_up_register_in_AMIGA_mode(changed_F2F3);
    }

    prev_joystate = _joystate;
  }
}

// PULLUP for F2BTN and F3BTN
void update_pull_up_register_in_AMIGA_mode(byte _changed_F2F3) {

#if defined(__7X_VER__)
  _changed_F2F3 >>= F3BTN; /*look at schematic of ver. 7.X :D,
                        F3BTN pin is followed
                        by F2BTN pin in PINB register,
                        and F3PULL_UP pin is followed by F2PULL_UP pin
                        in PINC register, but pins numbers are shifted by 4
                        */
#endif

  PINC = _changed_F2F3;  //hardware XOR
                         /* both PULLUP PINS are since setup()
                            reversed to F2BTN/F3BTN -- they are in
                            antiphase thus when F2 is being changed 1->0, the same
                            time F2_PULLUP will changed 0->1 :) 
                            antiphase is set up by set_C64_AMIGA_MODE_in_setup*/
}


byte ctl_on_flag_blinking(byte _ledstate) {
  if (1 ^ ctl_on_flag) {  //blinking ALL_LEDS if ctl_on_flag = 0 (no gamepad is connected)
    _ledstate = blinking_led ? bit(MODELED) : 0;
  }
  return _ledstate;
}

void push_ledstate_to_register(byte _ledstate) {
  //Pushing ledstate to PORTC

  static byte prev_ledstate;
  byte changed_ledstate = prev_ledstate ^ _ledstate;
  if (changed_ledstate) {     //it is changed by SEGA controller + EPROM read from setup()
    PINC = changed_ledstate;  //hardware XOR
    prev_ledstate = _ledstate;
  }
}

byte blinking_autofire_led(bool __switch__, byte __ledstate) {
  //blinking AUTORIRE LED if autofire in "ON"
  if (__switch__) {
    bitWrite(__ledstate, AUTOFIRELED, blinking_led);
  }
  return __ledstate;
}

bool nod_bool(bool OUT_DIR, bool SECOND_DIR) {
  //bool value = ((!( OUT_DIR  || SECOND_DIR )) || OUT_DIR); //logic where LOW=pressed, HIGH=released
  bool value = ((!(OUT_DIR && SECOND_DIR)) && OUT_DIR);  //logic where HIGH=pressed, LOW=released
  return value;
}

void process_state_controller(word current_gamepad_state) {
  ///////////////////////////////////////////////////////////
  //here everything happens only if current_state is changed
  ///////////////////////////////////////////////////////////
  word changed_gamepad_state = prev_gamepad_state ^ current_gamepad_state;
  if (changed_gamepad_state) {
    prev_gamepad_state = current_gamepad_state;

    for (byte index = 0; index < 15; index++) {
      if (changed_gamepad_state & 1) {
        button(index, current_gamepad_state & 1);
      } else if (!changed_gamepad_state) {
        break;
      }
      changed_gamepad_state >>= 1;
      current_gamepad_state >>= 1;
    }

    if (reset_flag) {
      reset_buttons();
      reset_flag = 0;
    }

    rapidfire_toggle(rapidfire_button | autofire_button | rapidfire_sw_button | rapid_left_right_btn | rapid_up_down_btn);

    //when autofire is "on" -- AUTOFIRE LED is on (steady light)
    if (1 ^ rapidfire_sw) {
      bitWrite(ledstate, AUTOFIRELED, autofire_button);
    }

    //join together DPAD_DOWN and BTN_DOWN -- BTN_UP prevails over DPAD_DOWN
    DPAD_DOWN = nod_bool(DPAD_DOWN, BTN_UP);

    //BTN_UP prevails over BTN_DOWN
    BTN_DOWN = nod_bool(BTN_DOWN, BTN_UP);

    //join together DPAD_UP and BTN_UP -- BTN_DOWN prevails over DPAD_UP
    DPAD_UP = nod_bool(DPAD_UP, BTN_DOWN);

    //starting timer is always related to pressing controller button
    //so execute timer_start() only if current_state is changed
    if (timer_start_flag) {
      timer_start(rapidfire_freq);
      timer_start_flag = 0;  //if timer is already started it would be futile to start it once again
    }
  }
}

void push_stuff() {
  ///////////////////////////////////////////////////////////
  //here everything happens in EVERY cycle of loop()
  ///////////////////////////////////////////////////////////
  bool tmp_fire_rapid = fire_rapid; /*since fire_rapid is updated by ISR(TIMER1_COMPA_vect)
                                      and that used more than once in this function it is prone to
                                      be changed during execution of the function.
                                      tmp_fire_rapid is "frozen" value of fire_rapid
                                    */

  bool fire_output = (rapidfire_button | autofire_button | rapidfire_sw_button) ? (fire_single | tmp_fire_rapid) : fire_single;
  set_btn(F1BTN, fire_output);
  /*fire_rapid is updated by ISR(TIMER1_COMPA_vect)
    thus update of F1BTN cannot depend upon
    change of SEGA controller buttons
    so the whole joystate PORT have to be updated
    in every possible cycle
  */

  if (rapid_up_down_btn) {
    bool BTN_UP_RAPID = tmp_fire_rapid;
    bool BTN_DOWN_RAPID = tmp_fire_rapid ^ 1;
    set_btn(UBTN, BTN_UP_RAPID);
    set_btn(DBTN, BTN_DOWN_RAPID);
  } else {
    set_btn(DBTN, DPAD_DOWN | BTN_DOWN);  //DOWN
    set_btn(UBTN, DPAD_UP | BTN_UP);      //UP
  }

  if (rapid_left_right_btn) {
    bool BTN_LEFT_RAPID = tmp_fire_rapid;
    bool BTN_RIGHT_RAPID = tmp_fire_rapid ^ 1;
    set_btn(LBTN, BTN_LEFT_RAPID);
    set_btn(RBTN, BTN_RIGHT_RAPID);
  } else {
    set_btn(RBTN, DPAD_RIGHT);
    set_btn(LBTN, DPAD_LEFT);
  }

  push_joystate_and_pullstate_to_register(joystate);

  push_ledstate_to_register(ctl_on_flag_blinking(blinking_autofire_led(rapidfire_sw, ledstate)));

  try_push_stuff_to_EEPROM();
}

/*
###################################################################
########################### EEPROM STUFF
###################################################################
*/

byte read_eeprom_stuff_index() {
  byte _tmp_eeprom_stuff_index = EEPROM.read(0);

  switch (_tmp_eeprom_stuff_index) {
    case 0:
      _tmp_eeprom_stuff_index = 1;
      break;
    default:
      if ((_tmp_eeprom_stuff_index - 1) % sizeof_eeprom_stuff) {  //there is remainder -- thus _tmp_eeprom_stuff_index IS NOT dividible by sizeof_eeprom_stuff
        _tmp_eeprom_stuff_index = 1;
      }
  }

  return _tmp_eeprom_stuff_index;
}

eeprom_buffer_struct read_eeprom_stuff_packed_data() {
  //read values from EEPROM
  eeprom_buffer_struct _tmp_eeprom_stuff;
  eeprom_stuff_index = read_eeprom_stuff_index();
  EEPROM.get(eeprom_stuff_index, _tmp_eeprom_stuff);
  return _tmp_eeprom_stuff;
}

void smart_eeprom_stuff_put() {
  if (eeprom_stuff._counter == 255) {
    switch (eeprom_stuff_index) {
      case 1 ... eeprom_stuff_last_before_max_index:
        eeprom_stuff_index += sizeof_eeprom_stuff;
        break;
      default:
        eeprom_stuff_index = 1;
    }
  }
  eeprom_stuff._counter++;
  noInterrupts();
  EEPROM.update(0, eeprom_stuff_index);
  EEPROM.put(eeprom_stuff_index, eeprom_stuff);
  interrupts();
}

byte pack_stuff_data(byte _ledstate, byte in_AMIGAmode__pullup_mode, byte in_rapidfire_freq) {
  in_rapidfire_freq <<= rapidfire_lbit;
  in_rapidfire_freq |= (_ledstate & (bit(CONF0LED) | bit(CONF1LED)));
  in_rapidfire_freq |= (in_AMIGAmode__pullup_mode & (bit(AMIGAmode_bit) | bit(pullup_mode_bit)));
  return in_rapidfire_freq;
}

void try_push_stuff_to_EEPROM() {
  if (save_eeprom_flag) {
    save_eeprom_flag = 0;
    byte _new_eeprom_stuff___packed_data = pack_stuff_data(ledstate, eeprom_stuff._packed_data, rapidfire_freq);
    if (eeprom_stuff._packed_data ^ _new_eeprom_stuff___packed_data) {
      eeprom_stuff._packed_data = _new_eeprom_stuff___packed_data;
      smart_eeprom_stuff_put();  //that takes time so before that eeprom_stuff's changes are checked earlier
    }
  }
}

//#########################################################################

void loop() {
  if (reading_controller_flag) {
    process_state_controller(sega.getState());
    reading_controller_flag = 0;
  }
  push_stuff();
  wdt_reset();
}

int main(void) {
  interrupts();

// Set internal oscillator prescaler if defined (in boards.txt)
#if defined(CLKPR) && defined(OSC_PRESCALER)
  CLKPR = 0x80;           // Enable prescaler
  CLKPR = OSC_PRESCALER;  // Set prescaler
#endif

  setup();

  while (1) {
    loop();
  }

  return 0;
}
