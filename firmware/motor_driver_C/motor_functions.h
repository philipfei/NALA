/*
 * motor_functions.h  --  motor power output (NALA_v0)
 *
 * Author  : HY (NALA v0 revision)
 * Version : v0.1.0
 * Date    : 2026-10-06
 */

#ifndef MOTOR_FUNCTIONS_H_
#define MOTOR_FUNCTIONS_H_

/*
 * Set the power of the four motors, each in -100..100 (sign = direction).
 * Positive = the wheel drives the robot forward (direction polarity is
 * handled inside, see motor_functions.c). Inputs must already be limited.
 */
void set_motor_power(int m1, int m2, int m3, int m4);

#endif /* MOTOR_FUNCTIONS_H_ */
