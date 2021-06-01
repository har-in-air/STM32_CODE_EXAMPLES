#ifndef MISC_H_
#define MISC_H_

#include <stdint.h>

#define MIN(a,b) (((a)<(b))?(a):(b))

#define HSE_VALUE    ((uint32_t)25000000) // external crystal oscillator
#define HSI_VALUE    ((uint32_t)16000000) // internal RC oscillator frequency

extern const uint8_t AHBPrescTable[];
extern const uint8_t APBPrescTable[];

void config_swdio_pins();
void system_core_clock_update();
uint32_t get_pclk1_freq(void);

#endif /* MISC_H_ */
