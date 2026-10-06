/*
 * motor_functions.c  --  motor power output
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06). Output logic is UNCHANGED from the
 * original; only includes/headers were completed, the unused K_p/K_i/K_d macros
 * (which would clash with config.h) were removed, and the direction pins now come
 * from config.h (MOTORn_DIR_BIT).
 *
 * Hardware: each motor has one PWM pin (timer output compare) and one direction
 * pin on PORTE:
 *      M1: PD6/OC0A + PE0     M2: PD5/OC0B + PE1
 *      M3: PB1/OC1A + PE2     M4: PB2/OC1B + PE3
 * The PWM outputs are in INVERTING mode (see pwm.c), hence the (100 - power) below.
 */

#include "config.h"
#include "motor_functions.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Drive the direction pin of one motor. */
static inline void set_dir_pin(uint8_t bit, int level)
{
	if (level) {
		PORTE |= (1 << bit);
	} else {
		PORTE &= ~(1 << bit);
	}
}

/**
 * @brief Set the the motor powers. Input should be given between -100..100.
 *
 * @param m1 Power for motor 1.
 * @param m2 Power for motor 2.
 * @param m3 Power for motor 3.
 * @param m4 Power for motor 4.
 *
 * Positive power -> direction pin LOW, negative or zero -> direction pin HIGH
 * (the original code negated the value first "for reversing the motor with position";
 * the result is identical).
 */
void set_motor_power(int m1, int m2, int m3, int m4) {
	set_dir_pin(MOTOR1_DIR_BIT, m1 <= 0);
	set_dir_pin(MOTOR2_DIR_BIT, m2 <= 0);
	set_dir_pin(MOTOR3_DIR_BIT, m3 <= 0);
	set_dir_pin(MOTOR4_DIR_BIT, m4 <= 0);

	if (m1 < 0) m1 = -m1;
	if (m2 < 0) m2 = -m2;
	if (m3 < 0) m3 = -m3;
	if (m4 < 0) m4 = -m4;

	//Be careful of integer / integer division.
	OCR0A = (int) ((100-m1)/100.0f * 0xFF); //PD6 (m1)
	OCR0B = (int) ((100-m2)/100.0f * 0xFF); //PD5 (m2)

	//Using the low byte register because we are using 8-bit PWM for the 16-bit timers
	OCR1AL = (int) ((100-m3)/100.0f * 0xFF); //PB1 (m3)
	OCR1BL = (int) ((100-m4)/100.0f * 0xFF); //PB2 (m4)
}
