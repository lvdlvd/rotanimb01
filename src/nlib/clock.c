#include "clock.h"

// The measured HSE frequency (0 = no crystal). Single definition so the rate
// queries and the init/measure functions share one value across translation
// units; clock_measure_hse() fills it.
uint32_t clock_hse_hz = 0;
