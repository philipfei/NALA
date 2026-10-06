/*
 * timer.h  --  timers (NALA_v0)
 *
 * Author  : HY (NALA v0 revision), original timer.c by Mathan
 * Version : v0.1.0
 * Date    : 2026-10-06
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

/*
 * Control-loop time base (NALA_v0): Timer3 in CTC mode, interrupt
 * TIMER3_COMPA_vect every CONTROL_PERIOD_MS (config.h). The ISR itself is in main.c.
 * Timer3 is therefore reserved for this; do not use start_timer3()/stop_timer3().
 */
void control_timer_init(void);

/* Original helpers, kept for reference. Never called by the firmware.
 * NOTE: start_timer3() selects CS32:0 = 111 (external clock on T3), not "no prescaler"
 * as its comment claims. */
void start_timer3(void);
uint16_t read_timer3(void);
void stop_timer3(void);
void start_timer4(void);
uint16_t read_timer4(void);
void stop_timer4(void);

#endif /* TIMER_H_ */
