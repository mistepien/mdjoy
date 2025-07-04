// FREQ0 = 7 CPS  (Quickgun Turbo Pro)
// FREQ1 = 13 CPS (Zipstick)
// FREQ2 = 29 CPS (Quickshot 128F)
// FREQ3 = 62 CPS (Competition Pro - Transparent model / Quickshot TopStar SV 127)

typedef struct {
  word ocr1a;
  byte prescaler;
} timer_configuration;

#if (F_CPU == 125000L)
 constexpr timer_configuration timer_params[4] = { {8927, 0}, {4806, 0}, {2154, 0}, {1007, 0} };
 //deviations: 0.000896453857421875 Hz, 0.00374412536621094 Hz, 0.004638671875 Hz, 0.0079345703125 Hz
#elif (F_CPU == 250000L)
 constexpr timer_configuration timer_params[4] = { {17856, 0}, {9614, 0}, {4309, 0}, {2015, 0} };
 //deviations: 0.000111579895019531 Hz, 0.00103950500488281 Hz, 0.004638671875 Hz, 0.0079345703125 Hz
#elif (F_CPU == 500000L)
 constexpr timer_configuration timer_params[4] = { {35713, 0}, {19229, 0}, {8619, 0}, {4031, 0} };
 //deviations: 0.000111579895019531 Hz, 0.00103950500488281 Hz, 0.004638671875 Hz, 0.0079345703125 Hz
#elif (F_CPU == 1000000L)
 constexpr timer_configuration timer_params[4] = { {8927, 1}, {38460, 0}, {17240, 0}, {8063, 0} };
 //deviations: 0.000896453857421875 Hz, 0.000364303588867188 Hz, 0.00127792358398438 Hz, 0.0079345703125 Hz
#elif (F_CPU == 1843200L)
 constexpr timer_configuration timer_params[4] = { {16456, 1}, {8860, 1}, {31778, 0}, {14863, 0} };
 //deviations: 0.000121116638183594 Hz, 0.00157928466796875 Hz, 0.0005645751953125 Hz, 0.004302978515625 Hz
#elif (F_CPU == 2000000L)
 constexpr timer_configuration timer_params[4] = { {17856, 1}, {9614, 1}, {34481, 0}, {16128, 0} };
 //deviations: 0.000111579895019531 Hz, 0.00103950500488281 Hz, 0.00127792358398438 Hz, 0.00025177001953125 Hz
#elif (F_CPU == 3276800L)
 constexpr timer_configuration timer_params[4] = { {29256, 1}, {15752, 1}, {56495, 0}, {26424, 0} };
 //deviations: 6.866455078125e-05 Hz, 0.00139617919921875 Hz, 0.0005645751953125 Hz, 0.0037841796875 Hz
#elif (F_CPU == 3686400L)
 constexpr timer_configuration timer_params[4] = { {32913, 1}, {17722, 1}, {63557, 0}, {29728, 0} };
 //deviations: 0.000121116638183594 Hz, 0.000112533569335938 Hz, 0.0005645751953125 Hz, 0.0001373291015625 Hz
#elif (F_CPU == 4000000L)
 constexpr timer_configuration timer_params[4] = { {35713, 1}, {19229, 1}, {8619, 1}, {32257, 0} };
 //deviations: 0.000111579895019531 Hz, 0.00103950500488281 Hz, 0.004638671875 Hz, 0.00025177001953125 Hz
#elif (F_CPU == 6000000L)
 constexpr timer_configuration timer_params[4] = { {53570, 1}, {28845, 1}, {12930, 1}, {48386, 0} };
 //deviations: 0.000111579895019531 Hz, 0.000139236450195312 Hz, 0.000156402587890625 Hz, 0.00025177001953125 Hz
#elif (F_CPU == 7372800L)
 constexpr timer_configuration timer_params[4] = { {8227, 2}, {35445, 1}, {15888, 1}, {59457, 0} };
 //deviations: 0.000972747802734375 Hz, 0.000112533569335938 Hz, 0.00239181518554688 Hz, 0.0001373291015625 Hz
#elif (F_CPU == 8000000L)
 constexpr timer_configuration timer_params[4] = { {8927, 2}, {38460, 1}, {17240, 1}, {64515, 0} };
 //deviations: 0.000896453857421875 Hz, 0.000364303588867188 Hz, 0.00127792358398438 Hz, 0.00025177001953125 Hz
#elif (F_CPU == 11059200L)
 constexpr timer_configuration timer_params[4] = { {12341, 2}, {53168, 1}, {23833, 1}, {11147, 1} };
 //deviations: 0.000972747802734375 Hz, 0.000112533569335938 Hz, 0.0011749267578125 Hz, 0.004302978515625 Hz
#elif (F_CPU == 12000000L)
 constexpr timer_configuration timer_params[4] = { {13391, 2}, {57691, 1}, {25861, 1}, {12095, 1} };
 //deviations: 0.000896453857421875 Hz, 0.000139236450195312 Hz, 0.000156402587890625 Hz, 0.0079345703125 Hz
#elif (F_CPU == 14745600L)
 constexpr timer_configuration timer_params[4] = { {16456, 2}, {8860, 2}, {31778, 1}, {14863, 1} };
 //deviations: 0.000121116638183594 Hz, 0.00157928466796875 Hz, 0.0005645751953125 Hz, 0.004302978515625 Hz
#elif (F_CPU == 16000000L)
 constexpr timer_configuration timer_params[4] = { {17856, 2}, {9614, 2}, {34481, 1}, {16128, 1} };
 //deviations: 0.000111579895019531 Hz, 0.00103950500488281 Hz, 0.00127792358398438 Hz, 0.00025177001953125 Hz
#elif (F_CPU == 18432000L)
 constexpr timer_configuration timer_params[4] = { {20570, 2}, {11075, 2}, {39723, 1}, {18579, 1} };
 //deviations: 0.000291824340820312 Hz, 0.002166748046875 Hz, 0.000202178955078125 Hz, 0.004302978515625 Hz
#elif (F_CPU == 20000000L)
 constexpr timer_configuration timer_params[4] = { {22320, 2}, {12018, 2}, {43102, 1}, {20160, 1} };
 //deviations: 0.000268936157226562 Hz, 0.000499725341796875 Hz, 0.00060272216796875 Hz, 0.0017852783203125 Hz
#endif
