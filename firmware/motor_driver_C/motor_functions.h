/*
 * motor_functions.h  --  motor power output (NALA_v1)
 *
 * Author  : HY (NALA v1 revision)
 * Version : v1.0.1
 * Date    : 2026-10-07
 */

#ifndef MOTOR_FUNCTIONS_H_
#define MOTOR_FUNCTIONS_H_

/*
 * Set the power of the four motors, each in percent -100.0 .. +100.0 (sign = direction).
 * Positive = the wheel drives the robot forward (direction polarity, including the
 * MOTORn_DIR_INVERT flags of config.h, is handled inside). Values must already be limited.
 * v1: float input and 8-bit duty resolution (1/255); v0 took integer percent (1 % steps).
 */
void set_motor_power(float m1, float m2, float m3, float m4);

#endif /* MOTOR_FUNCTIONS_H_ */
