/*
 * motor_functions.c  --  motor power output
 *
 * NALA_v1 revision (HY, v1.0.1, 2026-10-07):
 *   - direction polarity per motor is configurable (MOTORn_DIR_INVERT in config.h);
 *     all flags are 0: the M2/M3 supply cables are wired opposite to M1/M4 in the
 *     hardware, so no software inversion is needed (see the HARDWARE NOTE in config.h);
 *   - power is a float percent and is converted to the 8-bit PWM duty with rounding
 *     (1/255 resolution). v0 used integer percent, i.e. 1 % steps (~0.25 rad/s of wheel
 *     speed), which is coarse enough to make an integral controller hunt by one step.
 * The PWM mapping itself is unchanged: duty = |power| * 2.55, OCR = 255 - duty
 * (the outputs are in inverting mode, see pwm.c).
 *
 * Hardware: each motor has one PWM pin (timer output compare) and one direction
 * pin on PORTE:
 *      M1: PD6/OC0A + PE0     M2: PD5/OC0B + PE1
 *      M3: PB1/OC1A + PE2     M4: PB2/OC1B + PE3
 */

#include "config.h"
#include "motor_functions.h"

#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* Drive the direction pin of one motor. */
static inline void set_dir_pin(uint8_t bit, uint8_t level)
{
	if (level) {
		PORTE |= (1 << bit);
	} else {
		PORTE &= ~(1 << bit);
	}
}

/* Direction pin level: HIGH for power <= 0 (as in v0), flipped when the motor is inverted. */
static inline uint8_t dir_level(float power, uint8_t invert)
{
	uint8_t level = (power <= 0.0f);
	return invert ? !level : level;
}

/* |power| in percent -> 8-bit duty 0..255 (rounded, limited). */
static inline uint8_t duty_from_power(float power)
{
	float d = fabsf(power) * 2.55f + 0.5f;
	if (d > 255.0f) d = 255.0f;
	return (uint8_t)d;
}

/**
 * @brief Set the motor powers. Input in percent, -100.0 .. +100.0.
 *
 * @param m1 Power for motor 1.
 * @param m2 Power for motor 2.
 * @param m3 Power for motor 3.
 * @param m4 Power for motor 4.
 */
void set_motor_power(float m1, float m2, float m3, float m4) {
	set_dir_pin(MOTOR1_DIR_BIT, dir_level(m1, MOTOR1_DIR_INVERT));
	set_dir_pin(MOTOR2_DIR_BIT, dir_level(m2, MOTOR2_DIR_INVERT));
	set_dir_pin(MOTOR3_DIR_BIT, dir_level(m3, MOTOR3_DIR_INVERT));
	set_dir_pin(MOTOR4_DIR_BIT, dir_level(m4, MOTOR4_DIR_INVERT));

	OCR0A  = 255 - duty_from_power(m1);   //PD6 (m1)
	OCR0B  = 255 - duty_from_power(m2);   //PD5 (m2)

	//Using the low byte register because we are using 8-bit PWM for the 16-bit timers
	OCR1AL = 255 - duty_from_power(m3);   //PB1 (m3)
	OCR1BL = 255 - duty_from_power(m4);   //PB2 (m4)
}
