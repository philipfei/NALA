/*
 * ADC.h  --  ADC helper declarations (NALA_v0)
 *
 * Author  : HY (NALA v0 revision), original ADC.c by FlorisvM
 * Version : v0.1.0
 * Date    : 2026-10-06
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

/* mode = 0 -> AVCC (5 V) reference, mode = 1 -> internal 1.1 V reference.
 * Also prints 'a' (mode 0) or 'b' (mode 1) plus a newline over the UART, as the original did. */
void adc_init(int mode);

/* Blocking single conversion on the given channel; returns the 10-bit result. */
int read_adc(uint8_t channel);

#endif /* ADC_H_ */
