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
  and it saves cycles since in code there is no need to bother about XOR.
  
  That is available also in VPORT ATTiny chips (like ATTiny416):
  
  VPORTA.IN:
  Bits 7:0 – IN[7:0] Input Value
  This bit field shows the state of the PORTx pins when the digital input buffer is enabled.
  Writing a ‘0’ to bit n in this bit field has no effect.
  Writing a ‘1’ to bit n in this bit field will toggle the corresponding bit in PORTx.OUT.
  */

/*Source of these classic rapidfire frequencies - great input of Nightshft
  (Nightshft, thank you very much!):
  https://eab.abime.net/showpost.php?p=1364366&postcount=3

FREQ0 = 7 CPS  (Quickgun Turbo Pro)
FREQ1 = 13 CPS (Zipstick)
FREQ2 = 29 CPS (Quickshot 128F)
FREQ3 = 62 CPS (Competition Pro - Transparent model / Quickshot TopStar SV 127)

*/

//#define __7X_VER__

#if !((F_CPU == 500000L) || (F_CPU == 1000000L))
#error This is designed only for 0.5MHz and 1MHz clock!!!
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

#define bitWrite(value, bit, bitvalue) \
  ((value) = ((value) & ~(1U << (bit))) | (-(!!(bitvalue)) & (1U << (bit))))
#define bitRead(value, bit) ((value)&_BV(bit))
#define bitSet(value, bit) ((value) |= _BV(bit))
#define bitClear(value, bit) ((value) &= ~_BV(bit))
#define bitToggle(value, bit) ((value) ^= _BV(bit))
#define COMPLEX_BOOL_VALUE(big_byte, tested_byte) \
  (((big_byte) & (tested_byte)) == (tested_byte))

constexpr word _c64_amiga_combination = bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_DPAD_DOWN) | bit(SC_DPAD_RIGHT);
constexpr word _pullup_combination = bit(SC_BTN_A) | bit(SC_BTN_C) | bit(SC_DPAD_UP) | bit(SC_DPAD_LEFT);
constexpr word _gamepad_possible_buttons = 0xFFFF & ~(bit(SC_MODE));

bool BTN_UP = 0;
bool BTN_DOWN = 0;

bool dpad[4];
#define DPAD_UP dpad[0]
#define DPAD_DOWN dpad[1]
#define DPAD_LEFT dpad[2]
#define DPAD_RIGHT dpad[3]

word prev_gamepad_state;
byte rapidfire_freq = 0;
bool timer_start_flag = 0;     //it will let to start timer together with update of PORT registers
volatile bool fire_rapid = 0;  //this variable is toggled by timer interrupt (0101010101)
volatile bool reading_controller_flag = 0;
volatile bool blinking_led = 0;
volatile bool save_eeprom_flag = 0;
bool ctl_on_flag = 0;
byte buttons = 0;
constexpr byte rapidfire_button = 7;     //state of rapid button
constexpr byte autofire_button = 2;      //"artificial" state of autofire_button
constexpr byte rapidfire_sw = 0;         //to jest "stan" przełączenia przycisku FIRE(zwykłego) w tryb rapidfire
constexpr byte rapidfire_sw_button = 3;  //to jest wciśnięcie przycisku FIRE będącego w trybie rapidfire
constexpr byte fire_single = 4;          //to jest zwykły fire wciśnięty przez usera -- wciśnięty to strzał i tyle
constexpr byte rapid_up_down_btn = 5;
constexpr byte rapid_left_right_btn = 6;
constexpr byte start_btn = 1;
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


/*
###################################################################
########################### EEPROM STUFF
###################################################################
*/

byte read_eeprom_stuff_index() {
  byte _tmp_eeprom_stuff_index = EEPROM.read(0);

  if (_tmp_eeprom_stuff_index == 0 || ((_tmp_eeprom_stuff_index - 1) % sizeof_eeprom_stuff) != 0) {
    return 1;
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
    if (eeprom_stuff_index <= eeprom_stuff_last_before_max_index) {
      eeprom_stuff_index += sizeof_eeprom_stuff;
    } else {
      eeprom_stuff_index = 1;
    }
  }

  eeprom_stuff._counter++;

  noInterrupts();
  EEPROM.update(0, eeprom_stuff_index);
  EEPROM.put(eeprom_stuff_index, eeprom_stuff);
  interrupts();
}

inline byte pack_stuff_data(byte _ledstate,
                            byte in_AMIGAmode__pullup_mode,
                            byte in_rapidfire_freq) {
  return (in_rapidfire_freq << rapidfire_lbit)
         | (_ledstate & (bit(CONF0LED) | bit(CONF1LED)))
         | (in_AMIGAmode__pullup_mode & (bit(AMIGAmode_bit) | bit(pullup_mode_bit)));
}

void try_push_stuff_to_EEPROM(bool force = 0) {
  if (!(save_eeprom_flag || force)) return;

  save_eeprom_flag = 0;

  byte _new_eeprom_stuff___packed_data =
    pack_stuff_data(ledstate, eeprom_stuff._packed_data, rapidfire_freq);

  if (force || (_new_eeprom_stuff___packed_data != eeprom_stuff._packed_data)) {
    eeprom_stuff._packed_data = _new_eeprom_stuff___packed_data;
    smart_eeprom_stuff_put();
  }
}

//#########################################################################


void setup() {
  DDRB = 0xFF;  //OUTPUT: PB0-7:
  //DDRB |= ATARI_JOY | bit(F2BTN) | bit(F3BTN) | bit(C64MODE);

  /*
  UP,DOWN,RIGHT,LEFT and F1BTN are in reversed logic -
  "not pressed" ("released") is 1, so OUTPUT is set to HIGH
  F2BTN and F3BTN are set to 0 */
  PINB = ATARI_JOY;  //PORTB is default to LOW, toggle to HIGH only ATARI_JOY

  //pull-up stuff
  DDRC = 0xFF;
  //DDRC |= ALL_LEDS | bit(F3PULLUP) | bit(F2PULLUP);  //set all to output
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

  _delay_ms(200);
  ///LOOP FOR C64-AMIGA/PULLUP combination
  while (reads_num < max_blinks) {

    if (__blinking_led__) {
      _ledstate_setup = tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(MODELED) | bit(AUTOFIRELED);
    } else {
      _ledstate_setup = 0;
    }

    push_ledstate_to_register(_ledstate_setup);

    if (reading_controller_flag) {
      _sega_state_in_setup = sega.getState();
      reading_controller_flag = 0;
      if (is_C64_AMIGA_mode_combination_pressed_in_setup(_sega_state_in_setup)) {
        bitSet(__mode_combination_pressed__, AMIGAmode_bit);
      } else if (is_pullup_mode_combination_pressed_in_setup(_sega_state_in_setup)) {
        bitSet(__mode_combination_pressed__, pullup_mode_bit);
      }
    }

    if (__mode_combination_pressed__) {
      break;
    }

    __blinking_led__ = blinking_led;
    reads_num += catcher_timer_flag(__blinking_led__);
  }
  ////end of LOOP for combination

  //set up pullup_mode and C64_AMIGA_MODE
  tmp_eeprom_stuff._packed_data ^= __mode_combination_pressed__;

  if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
    set_C64_AMIGA_MODE_in_setup(bitRead(tmp_eeprom_stuff._packed_data, AMIGAmode_bit));
  }

  set_pullup_mode_in_setup(tmp_eeprom_stuff._packed_data);
  //////////////////////////////////////////

  eeprom_stuff = tmp_eeprom_stuff;

  //set joyconf
  byte tmp_joyconf = (tmp_eeprom_stuff._packed_data & (bit(CONF0LED) | bit(CONF1LED))) >> CONF0LED;
  set_joyconf(tmp_joyconf);

  //set rapidfire frequency
  rapidfire_freq = (tmp_eeprom_stuff._packed_data & (bit(rapidfire_hbit) | bit(rapidfire_lbit))) >> rapidfire_lbit;

  if (__mode_combination_pressed__) {
    try_push_stuff_to_EEPROM(1);
    if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
      blinking_leds_after_combination(ALL_LEDS);
    } else if (__mode_combination_pressed__ & bit(pullup_mode_bit)) {
      blinking_leds_after_combination(tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(MODELED) | bit(AUTOFIRELED));
    }
  }

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
  if (COMPLEX_BOOL_VALUE(___AMIGAmode___pullup_mode___, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
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

  // CTC && Prescaler 64
  TCCR0A = bit(CTC0) | bit(CS01) | bit(CS00);
#elif (F_CPU == 500000L)
  // 270.56277056277054 Hz (500000/((230+1)*8))
  OCR0A = 230;

  // CTC && Prescaler 8
  TCCR0A = bit(CTC0) | bit(CS01);
#endif

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

  // Prescaler 32
  TCCR2B = bit(CS21) | bit(CS20);
#elif (F_CPU == 500000L)
  // 270.56277056277054 Hz (500000/((230+1)*8))
  OCR2A = 230;

  // Prescaler 8
  TCCR2B = bit(CS21);
#endif

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
  _counter_eeprom = _counter_eeprom > 511 ? 0 : _counter_eeprom + 1;  //512 * 3,7ms = 1894.4 ms = 1.9s

  if (_counter_eeprom == 0) {
    save_eeprom_flag = 1;
  }

  static byte _counter_blinking;
  if (bitRead(buttons, rapidfire_sw) | (1 ^ ctl_on_flag)) {
    _counter_blinking = _counter_blinking > 40 ? 0 : _counter_blinking + 1;  //40 * 3,7ms = 148ms

    if (_counter_blinking == 0) {
      blinking_led ^= 1;
    }
  } else {
    blinking_led = 0;
  }
}

inline void timer_start(byte rspeed = 0) {
  noInterrupts();

  // ===== CACHE POINTER =====
  const auto* p = &timer_params[rspeed];

  // ===== SET MODE + OCR =====
  OCR1A = p->ocr1a;

#if (F_CPU == 1000000L)

  // precompute full TCCR1B value (no |= later)
  TCCR1B = bit(WGM12) | (p->prescaler ? bit(CS11) : bit(CS10));

#elif (F_CPU == 500000L)

  TCCR1B = bit(WGM12) | bit(CS10);

#endif

  // ===== ENABLE IRQ =====
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

    timer_start_flag = state;
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
  buttons = buttons & ~(bit(fire_single) | bit(rapidfire_button) | bit(rapid_up_down_btn) | bit(rapid_left_right_btn) | bit(autofire_button) | bit(start_btn));
  bitClear(joystate, F2BTN);
  bitClear(joystate, F3BTN);
  BTN_UP = 0;
  BTN_DOWN = 0;
  prev_gamepad_state &= (bit(SC_CTL_ON) | bit(SC_MODE) | bit(SC_DPAD_UP) | bit(SC_DPAD_DOWN) | bit(SC_DPAD_LEFT) | bit(SC_DPAD_RIGHT) | bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_BTN_HOME));
  //prev_gamepad_state &= (bit(SC_CTL_ON) | bit(SC_MODE) | bit(SC_BTN_A) | bit(SC_BTN_B) | bit(SC_BTN_HOME));
}

void btn_joyconf_toggle(bool btn_state) {
  if (btn_state) {
    set_joyconf(((ledstate & (bit(CONF0LED) | bit(CONF1LED))) >> CONF0LED) + 1);
  }
}

void set_btn(byte btn, bool btn_state) {
  bitWrite(joystate, btn, btn_state);
}

enum action_numbers {
  UP_BTN = 1,
  DOWN_BTN = 2,
  RAPIDFIRE = 3,
  AUTOFIRE = 4,
  F2 = 5,
  F3 = 6,
  UP_DOWN_RAPIDFIRE = 7,
  LEFT_RIGHT_RAPIDFIRE = 8,
  TOGGLE_RAPIDFIRE_FREQ = 9,
  TOGGLE_CONF = 10
};

/* that button_table row generator is based upon assumption on order of button in read:
enum sega_state {
  SC_CTL_ON = 0,  // The controller is connected
  SC_MODE = 1,    //checking six or three mode 1
  SC_BTN_START = 2,
  SC_DPAD_UP = 3,
  SC_DPAD_DOWN = 4,
  SC_DPAD_LEFT = 5,
  SC_DPAD_RIGHT = 6,
  SC_BTN_A = 7,
  SC_BTN_B = 8,
  SC_BTN_C = 9,
  SC_BTN_X = 10,
  SC_BTN_Y = 11,
  SC_BTN_Z = 12,
  SC_BTN_MODE = 13,
  SC_BTN_HOME = 14  //availabe in M30 2.4 8Bitdo gamepad
};
*/

// ==========================================================
// Row generator (fixed layout, no mistakes possible)
// ==========================================================
#define CONF_ROW(A, C, X, Y, Z) \
  { \
    0, 0, 0, 0, 0, 0, 0, \
      (A), /* [7] BTN_A */ \
      0, \
      (C),                   /* [9] BTN_C */ \
      (X),                   /* [10] BTN_X */ \
      (Y),                   /* [11] BTN_Y */ \
      (Z),                   /* [12] BTN_Z */ \
      TOGGLE_RAPIDFIRE_FREQ, /*MODE BTN*/ \
      TOGGLE_CONF            /*HOME BTN*/ \
  }

#define BTN_STRIDE 16  // power of two
const byte button_table[8][BTN_STRIDE] __attribute__((aligned(16))) = {
  // CONF 0
  CONF_ROW(F2, UP_BTN, 0, 0, 0),

  // CONF 1
  CONF_ROW(RAPIDFIRE, UP_BTN, 0, 0, 0),

  // CONF 2
  CONF_ROW(RAPIDFIRE, F2, 0, 0, 0),

  // CONF 3
  CONF_ROW(LEFT_RIGHT_RAPIDFIRE, UP_DOWN_RAPIDFIRE, 0, 0, 0),

  // CONF 4
  CONF_ROW(RAPIDFIRE, UP_BTN, AUTOFIRE, F2, F3),

  // CONF 5
  CONF_ROW(UP_BTN, RAPIDFIRE, F3, F2, AUTOFIRE),

  // CONF 6
  CONF_ROW(RAPIDFIRE, DOWN_BTN, F3, F2, UP_BTN),

  // CONF 7
  CONF_ROW(RAPIDFIRE, LEFT_RIGHT_RAPIDFIRE, F3, F2, UP_DOWN_RAPIDFIRE)
};

inline byte button_conf(byte _conf_num, byte btn) {
  return *((byte*)button_table + (_conf_num << 4) + btn);
}

inline __attribute__((always_inline)) void button(byte btn, bool btn_state) {

  // ==========================================================
  // HOTTEST: EARLY EXIT
  // ==========================================================

  if (btn == SC_BTN_START) {
    bitWrite(buttons, start_btn, btn_state);
    return;
  }

  if (btn == SC_CTL_ON) {
    reset_flag = 1;
    ctl_on_flag = btn_state;
    return;
  }

  if (btn == SC_MODE) {
    reset_flag = 1;
    bitClear(buttons, rapidfire_sw);
    bitClear(buttons, rapidfire_sw_button);
    bitWrite(ledstate, MODELED, btn_state);
    return;
  }

  // ==========================================================
  // CACHE (same naming style)
  // ==========================================================

  byte buttons_cache = buttons;
  byte ledstate_cache = ledstate;

  bool start_pressed = bitRead(buttons_cache, start_btn);
  bool mode_led = bitRead(ledstate_cache, MODELED);

  byte conf = (ledstate_cache >> CONF0LED) & 7;
  byte btn_action = button_conf(conf, btn);

  // ==========================================================
  // HOT PATH (NO START)
  // ==========================================================

  if (!start_pressed) {

    if (btn == SC_BTN_B) {
      byte target = bitRead(buttons_cache, rapidfire_sw)
                      ? rapidfire_sw_button
                      : fire_single;
      bitWrite(buttons, target, btn_state);
    } else if (btn <= SC_DPAD_RIGHT) {
      dpad[btn - SC_DPAD_UP] = btn_state;
    }

    action_func(btn_action, btn_state);
    return;
  }

  // ==========================================================
  // COLD PATH (START HELD)
  // ==========================================================

  // -------- PRESS --------
  if (btn_state) {

    // DPAD (dense → switch is optimal)
    if (btn <= SC_DPAD_RIGHT) {
      switch (btn) {
        case SC_DPAD_UP: rapidfire_freq = 0; return;
        case SC_DPAD_RIGHT: rapidfire_freq = 1; return;
        case SC_DPAD_DOWN: rapidfire_freq = 2; return;
        case SC_DPAD_LEFT: rapidfire_freq = 3; return;
      }
    }

    switch (btn) {

      case SC_BTN_A:
        btn_joyconf_toggle(1);
        return;

      case SC_BTN_B:
        {
          bool rapidfire_sw_old = bitRead(buttons_cache, rapidfire_sw);

          if (!mode_led) {
            bitToggle(buttons, rapidfire_sw);

            if (rapidfire_sw_old)
              bitClear(buttons, rapidfire_sw_button);
            else
              bitClear(buttons, autofire_button);

          } else {
            bitSet(buttons, fire_single);
          }
          return;
        }

      case SC_BTN_C:
        set_rapidfire_freq(rapidfire_freq + 1);
        return;
    }

    return;  // no default action on press
  }

  // -------- RELEASE --------

  if (btn <= SC_DPAD_RIGHT) {
    dpad[btn - SC_DPAD_UP] = 0;
    return;
  }

  switch (btn) {
    case SC_BTN_B:
      if (mode_led)
        bitClear(buttons, fire_single);
      break;
  }

  // IMPORTANT: always propagate release (bug fix)
  action_func(btn_action, btn_state);
}

inline void action_func(byte _num, bool btn_state) {

  switch (_num) {

    case RAPIDFIRE:
      bitWrite(buttons, rapidfire_button, btn_state);
      break;

    case UP_BTN:
      BTN_UP = btn_state;
      break;

    case DOWN_BTN:
      BTN_DOWN = btn_state;
      break;

    case UP_DOWN_RAPIDFIRE:
      bitWrite(buttons, rapid_up_down_btn, btn_state);
      break;

    case LEFT_RIGHT_RAPIDFIRE:
      bitWrite(buttons, rapid_left_right_btn, btn_state);
      break;

    case F2:
      set_btn(F2BTN, btn_state);
      break;

    case F3:
      set_btn(F3BTN, btn_state);
      break;

    case AUTOFIRE:
      if (btn_state) {
        bitToggle(buttons, autofire_button);

        if (bitRead(buttons, autofire_button)) {
          bitClear(buttons, rapidfire_sw);

          if (bitRead(buttons, rapidfire_sw_button)) {
            bitClear(buttons, rapidfire_sw_button);
            bitSet(buttons, fire_single);
          }
        }
      }
      break;

    case TOGGLE_RAPIDFIRE_FREQ:
      if (btn_state) set_rapidfire_freq(rapidfire_freq + 1);
      break;

    case TOGGLE_CONF:
      if (btn_state) btn_joyconf_toggle(1);
      break;
  }
}

void push_joystate_and_pullstate_to_register(byte _joystate) {
  //Pushing joystate to PORTB
  static byte prev_joystate;
  byte changed_joystate = prev_joystate ^ _joystate;
  if (changed_joystate) {
    PINB = changed_joystate;  //hardware XOR

    if (COMPLEX_BOOL_VALUE(eeprom_stuff._packed_data, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
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


inline byte ctl_on_flag_blinking(byte _ledstate) {
  if (!ctl_on_flag) {  //blinking ALL_LEDS if ctl_on_flag = 0 (no gamepad is connected)
    return blinking_led ? bit(MODELED) : 0;
  }
  return _ledstate;
}

inline void push_ledstate_to_register(byte _ledstate) {
  //Pushing ledstate to PORTC

  static byte prev_ledstate;
  byte changed_ledstate = prev_ledstate ^ _ledstate;

  if (!changed_ledstate) return;

  PINC = changed_ledstate;  //hardware XOR
  prev_ledstate = _ledstate;
}

byte blinking_autofire_led(bool __switch__, byte __ledstate) {
  //blinking AUTORIRE LED if autofire in "ON"
  if (__switch__) {
    bitWrite(__ledstate, AUTOFIRELED, blinking_led);
  }
  return __ledstate;
}

bool priority(bool OUT_DIR, bool SECOND_DIR) {
  bool value = ((!(OUT_DIR && SECOND_DIR)) && OUT_DIR);  //logic where HIGH=pressed, LOW=released
  return value;
}

inline void process_state_controller(word current_gamepad_state) {
  ///////////////////////////////////////////////////////////
  //here everything happens only if current_state is changed
  ///////////////////////////////////////////////////////////
  word changed_gamepad_state = prev_gamepad_state ^ current_gamepad_state;
  if (!changed_gamepad_state) return;

  prev_gamepad_state = current_gamepad_state;

  // process only changed bits
  while (changed_gamepad_state) {
    byte index = __builtin_ctz(changed_gamepad_state);
    button(index, (current_gamepad_state >> index) & 1);
    changed_gamepad_state &= (changed_gamepad_state - 1);
  }

  if (reset_flag) {
    reset_buttons();
    reset_flag = 0;
  }

  rapidfire_toggle(
    buttons & (bit(rapidfire_button) | bit(autofire_button) | bit(rapidfire_sw_button) | bit(rapid_left_right_btn) | bit(rapid_up_down_btn)));

  //when autofire is "on" -- AUTOFIRE LED is on (steady light)
  if (1 ^ bitRead(buttons, rapidfire_sw)) {
    bitWrite(ledstate, AUTOFIRELED, bitRead(buttons, autofire_button));
  }

  //join together DPAD_DOWN and BTN_DOWN -- BTN_UP prevails over DPAD_DOWN
  DPAD_DOWN = priority(DPAD_DOWN, BTN_UP);

  //BTN_UP prevails over BTN_DOWN
  BTN_DOWN = priority(BTN_DOWN, BTN_UP);

  //join together DPAD_UP and BTN_UP -- BTN_DOWN prevails over DPAD_UP
  DPAD_UP = priority(DPAD_UP, BTN_DOWN);

  //starting timer is always related to pressing controller button
  if (timer_start_flag) {
    timer_start(rapidfire_freq);
    timer_start_flag = 0;
  }
}

inline void push_stuff() {
  ///////////////////////////////////////////////////////////
  //here everything happens in EVERY cycle of loop()
  ///////////////////////////////////////////////////////////

  bool tmp_fire_rapid = fire_rapid;

  // ===== CACHE =====
  byte _buttons = buttons;
  byte _ledstate = ledstate;

  bool _fire_single = bitRead(_buttons, fire_single);
  bool _rapidfire_sw = bitRead(_buttons, rapidfire_sw);
  bool _autofire = bitRead(_buttons, autofire_button);
  bool _rapid_ud = bitRead(_buttons, rapid_up_down_btn);
  bool _rapid_lr = bitRead(_buttons, rapid_left_right_btn);

  // ===== FIRE =====
  bool fire_output =
    (_buttons & (bit(rapidfire_button) | bit(autofire_button) | bit(rapidfire_sw_button)))
      ? (_fire_single | tmp_fire_rapid)
      : _fire_single;

  set_btn(F1BTN, fire_output);

  // ===== UP / DOWN =====
  if (_rapid_ud) {
    set_btn(UBTN, tmp_fire_rapid);
    set_btn(DBTN, !tmp_fire_rapid);
  } else {
    set_btn(DBTN, DPAD_DOWN | BTN_DOWN);
    set_btn(UBTN, DPAD_UP | BTN_UP);
  }

  // ===== LEFT / RIGHT =====
  if (_rapid_lr) {
    set_btn(LBTN, tmp_fire_rapid);
    set_btn(RBTN, !tmp_fire_rapid);
  } else {
    set_btn(RBTN, DPAD_RIGHT);
    set_btn(LBTN, DPAD_LEFT);
  }

  push_joystate_and_pullstate_to_register(joystate);

  // ===== LED =====
  if (!_rapidfire_sw) {
    bitWrite(_ledstate, AUTOFIRELED, _autofire);
  }

  if (!ctl_on_flag) {
    _ledstate = blinking_led ? bit(MODELED) : 0;
  }

  push_ledstate_to_register(_ledstate);

  try_push_stuff_to_EEPROM();
}

void loop() {
  if (reading_controller_flag) {
    process_state_controller(sega.getState());
    reading_controller_flag = 0;
  }
  push_stuff();
  wdt_reset();
}

int main(void) {

// Set internal oscillator prescaler if defined (in boards.txt)
#if defined(CLKPR) && defined(OSC_PRESCALER)
  CLKPR = 0x80;           // Enable prescaler
  CLKPR = OSC_PRESCALER;  // Set prescaler
#endif

  setup();

  interrupts();

  for (;;) {
    loop();
  }

  return 0;
}