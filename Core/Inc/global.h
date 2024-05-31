#include <cstdint>

#ifndef GLOBAL_H
#define GLOBAL_H

// Declare the global variable
extern bool reset_step_sample, run;
extern uint8_t rxByte;
extern uint16_t step_sample;
extern uint8_t step;
extern uint8_t bpm_source[3];

#endif // GLOBAL_H