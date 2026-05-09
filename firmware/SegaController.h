// Author:
//       Michał Stępień <mistepien@wp.pl>

#if defined(ARDUINO_AVR_MICRO)

#define DDR_REG_select_Pin DDRD
#define PORT_REG_selectPin PORTD
#define PIN_REG_selectPin PIND

#define DDR_REG_inputPins DDRB
#define PORT_REG_inputPins PORTB
#define PIN_REG_inputPins PINB

#else

#define DDR_REG_select_Pin DDRD
#define PORT_REG_selectPin PORTD
#define PIN_REG_selectPin PIND

#define DDR_REG_inputPins DDRD
#define PORT_REG_inputPins PORTD
#define PIN_REG_inputPins PIND

#endif

// ==========================================================
// TIMING CONFIG (MOVED HERE)
// ==========================================================

#if (F_CPU == 500000L)
#define SEGA_DELAY_CYCLES 6
#elif (F_CPU == 1000000L)
#define SEGA_DELAY_CYCLES 12
#else
#error "Unsupported F_CPU"
#endif

// ==========================================================
// STATE ENUM
// ==========================================================

enum sega_state {
  SC_CTL_ON = 0,
  SC_MODE = 1,
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
  SC_BTN_HOME = 14
};

// ==========================================================
// BIT MASKS
// ==========================================================

enum {
  B_SC_BTN_A = bit(SC_BTN_A),
  B_SC_BTN_START = bit(SC_BTN_START),
  B_SC_CTL_ON = bit(SC_CTL_ON),
  B_SC_DPAD_UP = bit(SC_DPAD_UP),
  B_SC_DPAD_DOWN = bit(SC_DPAD_DOWN),
  B_SC_DPAD_LEFT = bit(SC_DPAD_LEFT),
  B_SC_DPAD_RIGHT = bit(SC_DPAD_RIGHT),
  B_SC_BTN_B = bit(SC_BTN_B),
  B_SC_BTN_C = bit(SC_BTN_C),
  B_SC_MODE = bit(SC_MODE),
  B_SC_BTN_Z = bit(SC_BTN_Z),
  B_SC_BTN_Y = bit(SC_BTN_Y),
  B_SC_BTN_X = bit(SC_BTN_X),
  B_SC_BTN_MODE = bit(SC_BTN_MODE),
  B_SC_BTN_HOME = bit(SC_BTN_HOME)
};

const word MASK_UD = B_SC_DPAD_UP | B_SC_DPAD_DOWN;
const word MASK_LR = B_SC_DPAD_LEFT | B_SC_DPAD_RIGHT;

// ==========================================================
// CLASS
// ==========================================================

class SegaController {
public:
  void begin(byte db9_pin_7, byte db9_pin_1, byte db9_pin_2,
             byte db9_pin_3, byte db9_pin_4,
             byte db9_pin_6, byte db9_pin_9);

  word getState();

private:
  byte _selectPin_bin;
  byte _inputPins[6];

  byte _mask_c2_btnA;
  byte _mask_c2_start;
  byte _mask_c2_ctl1;
  byte _mask_c2_ctl2;
  byte _mask_c2_ctl;

  byte _mask_c3_up;
  byte _mask_c3_down;
  byte _mask_c3_left;
  byte _mask_c3_right;
  byte _mask_c3_b;
  byte _mask_c3_c;

  byte _mask_c4_mode1;
  byte _mask_c4_mode2;
  byte _mask_c4_mode;

  byte _mask_c5_z;
  byte _mask_c5_y;
  byte _mask_c5_x;
  byte _mask_c5_mode;

  byte _mask_c6_home;

  inline void init_cycle_masks();
  inline word build_state_fast_raw(const byte* _readCycle_regs);
};

extern SegaController sega;