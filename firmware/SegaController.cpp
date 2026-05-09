/*********************************************
   Created by Michał Stępień
   Contact: mistepien@wp.pl
   License: GPLv3
 *********************************************/

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

  ------------------
  |  HARDWARE XOR  |
  ------------------
  PINx = byte;      <======>    PORTx ^= byte;

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



  attiny416 (PORTx version):

  // Set PA3 as OUTPUT:
PORTA.DIRSET = (1 << 3);

// Set PA3 as INPUT:
PORTA.DIRCLR = (1 << 3);

// Enable pull-up on PA3 (bit 3 in PIN3CTRL):
PORTA.PIN3CTRL |= (1 << 3);

// Disable pull-up on PA3:
PORTA.PIN3CTRL &= ~(1 << 3);

// Set PA3 HIGH:
PORTA.OUTSET = (1 << 3);

// Set PA3 LOW:
PORTA.OUTCLR = (1 << 3);

// Toggle PA3:
PORTA.OUTTGL = (1 << 3);

// Read PA3:
uint8_t v = PORTA.IN & (1 << 3);

// If PA3 is HIGH:
if (PORTA.IN & (1 << 3)) {}

// If PA3 is LOW:
if (!(PORTA.IN & (1 << 3))) {}



attiny416 (VPORTs version)

// Set PA3 as OUTPUT:
VPORTA.DIR |= (1 << 3);

// Set PA3 as INPUT:
VPORTA.DIR &= ~(1 << 3);

// Enable pull-up on PA3:
PORTA.PIN3CTRL |= (1 << 3);

// Disable pull-up on PA3:
PORTA.PIN3CTRL &= ~(1 << 3);

// Set PA3 HIGH:
VPORTA.OUT |= (1 << 3);

// Set PA3 LOW:
VPORTA.OUT &= ~(1 << 3);

// Toggle PA3:
VPORTA.IN |= (1 << 3);

// Read PA3:
uint8_t v = VPORTA.IN & (1 << 3);

// If PA3 is HIGH:
if (VPORTA.IN & (1 << 3)) {}

// If PA3 is LOW:
if (!(VPORTA.IN & (1 << 3))) {}


Key points (why VPORT)

  VPORTA behaves like classic AVR (PORTA, DDRA, etc.)
  single-cycle access (fast, like ATmega)
  still compatible with (1 << n) style
  Only difference: pull-up lives outside VPORT

 -- VPORT vs PORT — what’s the real difference?
  1. VPORT (Virtual PORT)
  Designed to behave like old ATmega-style registers

Registers:
VPORTA.DIR
VPORTA.OUT
VPORTA.IN

✔ Single-cycle access (very fast)
✔ Supports classic bit ops:

VPORTA.OUT |= (1 << 3);
VPORTA.OUT &= ~(1 << 3);

✔ Maps to CPU I/O space → efficient instructions

 2. PORT (Full-featured peripheral)

 Modern, feature-rich GPIO system

Registers:
PORTA.DIRSET, DIRCLR
PORTA.OUTSET, OUTCLR, OUTTGL
PORTA.IN
PORTA.PINnCTRL ← config per pin

✔ Atomic operations (no read-modify-write issues)
✔ Per-pin configuration (pull-up, interrupt, inversion, etc.)
✔ More flexible and safer

⚡ Side-by-side example
Using VPORT (classic style)
VPORTA.DIR |= (1 << 3);
VPORTA.OUT |= (1 << 3);

Using PORT (modern style)
PORTA.DIRSET = (1 << 3);
PORTA.OUTSET = (1 << 3);

 Both do the same thing
 But the second is atomic and safer

atmega48:
// Set PA3 as OUTPUT:
DDRA |= (1 << 3);

// Set PA3 as INPUT:
DDRA &= ~(1 << 3);

// Enable pull-up on PA3 (must be input):
DDRA &= ~(1 << 3); PORTA |= (1 << 3);

// Disable pull-up on PA3:
PORTA &= ~(1 << 3);

// Set PA3 HIGH (output):
PORTA |= (1 << 3);

// Set PA3 LOW (output):
PORTA &= ~(1 << 3);

// Toggle PA3:
PINA |= (1 << 3);

// Read PA3:
uint8_t v = PINA & (1 << 3);

// If PA3 is HIGH:
if (PINA & (1 << 3)) {}

// If PA3 is LOW:
if (!(PINA & (1 << 3))) {}

  */


#include "Arduino.h"
#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
#include <util/delay.h>
#endif
#include "SegaController.h"

// ==========================================================
// INIT
// ==========================================================

void SegaController::begin(byte db9_pin_7, byte db9_pin_1, byte db9_pin_2,
                           byte db9_pin_3, byte db9_pin_4,
                           byte db9_pin_6, byte db9_pin_9) {

  byte _selectPin = db9_pin_7;
  _selectPin_bin = bit(_selectPin);

  bitSet(DDR_REG_select_Pin, _selectPin);
  bitSet(PORT_REG_selectPin, _selectPin);

  _inputPins[0] = bit(db9_pin_1);
  _inputPins[1] = bit(db9_pin_2);
  _inputPins[2] = bit(db9_pin_3);
  _inputPins[3] = bit(db9_pin_4);
  _inputPins[4] = bit(db9_pin_6);
  _inputPins[5] = bit(db9_pin_9);

#if defined(ARDUINO_AVR_MICRO)
  byte _all_inputPins = 0;
  for (byte i = 0; i < SC_INPUT_PINS; i++) {
    _all_inputPins |= _inputPins[i];
  }
  PORT_REG_inputPins |= _all_inputPins;
#endif

  init_cycle_masks();
}

// ==========================================================
// MASK INIT
// ==========================================================

inline void SegaController::init_cycle_masks() {

  _mask_c2_btnA = _inputPins[4];
  _mask_c2_start = _inputPins[5];
  _mask_c2_ctl1 = _inputPins[2];
  _mask_c2_ctl2 = _inputPins[3];

  _mask_c3_up = _inputPins[0];
  _mask_c3_down = _inputPins[1];
  _mask_c3_left = _inputPins[2];
  _mask_c3_right = _inputPins[3];
  _mask_c3_b = _inputPins[4];
  _mask_c3_c = _inputPins[5];

  _mask_c4_mode1 = _inputPins[0];
  _mask_c4_mode2 = _inputPins[1];

  _mask_c5_z = _inputPins[0];
  _mask_c5_y = _inputPins[1];
  _mask_c5_x = _inputPins[2];
  _mask_c5_mode = _inputPins[3];

  _mask_c6_home = _inputPins[0];

  _mask_c2_ctl = _mask_c2_ctl1 | _mask_c2_ctl2;
  _mask_c4_mode = _mask_c4_mode1 | _mask_c4_mode2;
}

// ==========================================================
// TEMPLATE UNROLLED READER
// ==========================================================

template<byte N>
inline void read_cycle(byte*& p, byte select_mask) {

  PIN_REG_selectPin = select_mask;

  __builtin_avr_delay_cycles(SEGA_DELAY_CYCLES);

  *p++ = PIN_REG_inputPins;

  if constexpr (N > 1)
    read_cycle<N - 1>(p, select_mask);
}

// ==========================================================
// GET STATE
// ==========================================================

word SegaController::getState() {

  byte _readCycle_regs[8] __attribute__((aligned(8)));

  noInterrupts();

  byte* p = _readCycle_regs;
  read_cycle<8>(p, _selectPin_bin);

  interrupts();

  return build_state_fast_raw(_readCycle_regs);
}

// ==========================================================
// FAST DECODE (STRICT MATCHING VERSION)
// ==========================================================

inline word SegaController::build_state_fast_raw(const byte* _readCycle_regs) {

  byte _cycle2 = ~_readCycle_regs[2];
  byte _cycle3 = ~_readCycle_regs[3];
  byte _cycle4 = ~_readCycle_regs[4];
  byte _cycle5 = ~_readCycle_regs[5];
  byte _cycle6 = ~_readCycle_regs[6];

  // ==========================================================
  // AUTO-RESET (controller unplugged)
  // ==========================================================

  byte _any = _cycle2 | _cycle3 | _cycle4;
  _any |= _cycle5 | _cycle6;
  if (!_any) return 0;

  word _state = 0;

  // ==========================================================
  // CYCLE 2
  // ==========================================================

  if ((_cycle2 & _mask_c2_btnA) == _mask_c2_btnA) _state |= B_SC_BTN_A;
  if ((_cycle2 & _mask_c2_start) == _mask_c2_start) _state |= B_SC_BTN_START;
  if ((_cycle2 & _mask_c2_ctl) == _mask_c2_ctl) _state |= B_SC_CTL_ON;

  // ==========================================================
  // CYCLE 3
  // ==========================================================

  if ((_cycle3 & _mask_c3_up) == _mask_c3_up) _state |= B_SC_DPAD_UP;
  if ((_cycle3 & _mask_c3_down) == _mask_c3_down) _state |= B_SC_DPAD_DOWN;
  if ((_cycle3 & _mask_c3_left) == _mask_c3_left) _state |= B_SC_DPAD_LEFT;
  if ((_cycle3 & _mask_c3_right) == _mask_c3_right) _state |= B_SC_DPAD_RIGHT;
  if ((_cycle3 & _mask_c3_b) == _mask_c3_b) _state |= B_SC_BTN_B;
  if ((_cycle3 & _mask_c3_c) == _mask_c3_c) _state |= B_SC_BTN_C;

  // ==========================================================
  // CYCLE 4 → MODE (STRICT)
  // ==========================================================

  byte _is6btn = ((_cycle4 & _mask_c4_mode) == _mask_c4_mode);
  _state |= _is6btn << SC_MODE;

  // ==========================================================
  // CYCLE 5 (STRICT)
  // ==========================================================

  if ((_cycle5 & _mask_c5_z) == _mask_c5_z) _state |= B_SC_BTN_Z;
  if ((_cycle5 & _mask_c5_y) == _mask_c5_y) _state |= B_SC_BTN_Y;
  if ((_cycle5 & _mask_c5_x) == _mask_c5_x) _state |= B_SC_BTN_X;
  if ((_cycle5 & _mask_c5_mode) == _mask_c5_mode) _state |= B_SC_BTN_MODE;

  // ==========================================================
  // CYCLE 6
  // ==========================================================

  if ((_cycle6 & _mask_c6_home) == _mask_c6_home) {
    _state |= B_SC_BTN_HOME;
  }
  // ==========================================================
  // EXTRA SAFETY (disable extended buttons if not 6btn)
  // ==========================================================

  if (!_is6btn) {
    _state &= ~(B_SC_BTN_HOME | B_SC_BTN_Z | B_SC_BTN_Y | B_SC_BTN_X | B_SC_BTN_MODE);
  }

  // ==========================================================
  // POST PROCESSING
  // ==========================================================

  // ---- cancel opposite directions ----
  if ((_state & MASK_UD) == MASK_UD) _state ^= MASK_UD;
  if ((_state & MASK_LR) == MASK_LR) _state ^= MASK_LR;

  // ---- apply SN_CTL_ON ----
  if (!(_state & B_SC_CTL_ON)) return 0;

  return _state;
}

// ==========================================================

SegaController sega;