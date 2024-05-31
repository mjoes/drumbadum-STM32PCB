#include "global.h"

bool reset_step_sample = true;
bool run = false;
uint16_t step_sample = 0;
uint8_t step = 0;
uint8_t rxByte;
uint8_t bpm_source[3] = { 120, 120, 120 };