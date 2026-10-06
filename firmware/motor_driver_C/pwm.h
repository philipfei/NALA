/*
 * pwm.h  --  PWM / motor pin initialisation (NALA_v0)
 *
 * Author  : HY (NALA v0 revision), original pwm.c by Mathan
 * Version : v0.1.0
 * Date    : 2026-10-06
 */

#ifndef PWM_H_
#define PWM_H_

/* Make PD6, PD5, PB1, PB2 (the four PWM outputs) outputs. */
void init_motor_pins(void);

/* Timer0 (PD5/PD6) and Timer1 (PB1/PB2): 8-bit inverting fast PWM, no prescaler. */
void init_motor_timers(void);

#endif /* PWM_H_ */
