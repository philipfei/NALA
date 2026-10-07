/*
 * timer.c
 *
 * Created: 12/21/2021 4:17:34 PM
 *  Author: Mathan
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06): added control_timer_init() (the
 * control-loop time base). The original start/read/stop helpers are unchanged
 * and still unused.
 */
#include "config.h"
#include "timer.h"

#include <avr/io.h>

/*
 * Timer3, CTC mode (WGM32), clock = F_CPU / 64 (CS31 + CS30).
 * With the default 50 ms period (v1): 250 kHz tick, OCR3A = 12499 (0x30D3).
 * (v0 used 100 ms, OCR3A = 24999.) The value is computed from CONTROL_PERIOD_MS in config.h.
 * The compare-match interrupt (TIMER3_COMPA_vect, in main.c) fires every period,
 * exactly, regardless of how long the main loop takes.
 */
void control_timer_init(void) {
    TCCR3A = 0;
    TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
    TCNT3  = 0;
    OCR3A  = (uint16_t) CONTROL_TIMER_OCR;
    TIMSK3 = _BV(OCIE3A);
}

void start_timer3() {
    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaling
    TCCR3B |= _BV(CS31); 
    TCCR3B |= (_BV(CS30) | _BV(CS32));
}

//Might change return type if necessary
uint16_t read_timer3() {
    return ((TCNT3H << 8) | TCNT3L);
}

void stop_timer3() {
    TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
}

/*
 * 
 *  Doing the same as above for timer4 
 * 
 */
void start_timer4() {
    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaling
    TCCR4B |= _BV(CS40); 
    TCCR4B &= ~(_BV(CS41) | _BV(CS42));
}

//Might change return type if necessary
uint16_t read_timer4() {
    return ((TCNT4H << 8) | TCNT4L);
}

void stop_timer4() {
    TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
}
