/*
 * main.c  --  NALA_v1 motor driver firmware
 *
 * Project  : NALA_v1 (ATmega328PB, 4 mecanum wheels, 4 quadrature encoders)
 * Author   : HY / Claude (NALA v1 revision)
 * Version  : v1.1.0
 * Date     : 2026-10-07
 *
 * v1 vs v0: 20 Hz control loop; speed controller replaced by a positional PI with
 * feed-forward and anti-windup (the v0 incremental PID rang); M2/M3 motor polarity
 * inverted in v1.0.0 and reverted in v1.0.1; command timeout 200 ms.
 * v1.1.0: the feedback line carries the cumulative encoder counts and is sent every
 * period (20 Hz); UART 38400 baud; feed-forward 6.5 %/(rad/s) (measured).
 *
 * Original : motor_driver_C by Floris van Mourik (created 9/18/2021), with pwm/timer
 *            code by Mathan and UART code by sojim. The untouched original sources
 *            are kept in ../original_ref/.
 *
 * ---------------------------------------------------------------------------
 * OVERVIEW
 *   Receives body velocity commands (Vx, Vy, w) over UART, converts them to four
 *   wheel speed targets with the mecanum inverse kinematics, and runs one speed
 *   controller (PI + feed-forward) per wheel on the speed measured from the
 *   encoders. Its output drives four PWM motor channels. The cumulative encoder
 *   counts of the four wheels are printed back over the UART (odometry on the host).
 *
 * STRUCTURE (no blocking loops, no software delays)
 *   - Encoders   : pin-change interrupts count edges continuously     (encoder.c)
 *   - Time base  : Timer3 compare interrupt, exactly every CONTROL_PERIOD_MS
 *                  (TIMER3_COMPA_vect below). It snapshots the encoder counts.
 *   - UART RX    : interrupt, only collects the 5-byte frame and sets a flag.
 *   - UART TX    : ring buffer drained by the UDRE interrupt             (USART.c)
 *   - main loop  : (1) apply a newly received command,
 *                  (2) when a control period has elapsed: compute speeds, run the
 *                      PI controllers, set the motor output, send telemetry.
 *
 * SPEED MEASUREMENT
 *   speed [rad/s] = (counts in the period / ENC_COUNTS_PER_REV) * 2*pi / period
 *   Because the period comes from a hardware timer (and is multiplied by the
 *   number of timer ticks actually elapsed, in case the main loop is ever late),
 *   the time base is exact. The original code assumed 0.010 s per measurement
 *   window and 0.045 s per loop; the real values were larger (print and delay
 *   included), which made measured speeds and the PID I/D terms inaccurate.
 *   At the default 50 ms period one encoder count is 0.082 rad/s.
 *
 * UART PROTOCOL (frame layout unchanged; scale now 0.02, see below)
 *   Host -> MCU, 38400 8N1: 0x80 0x86 Vx Vy w   (3x int8; /50 -> m/s, m/s, rad/s; 0.02 per count)
 *   NOTE: the original scale was /100. The host (Pi) must send value*50.
 *   Axes: ROS convention, Vx forward, Vy left, w counter-clockwise positive.
 *   Safety: no velocity frame for CMD_TIMEOUT_MS (v1: 200 ms) while moving -> stop,
 *   MCU prints "cmd timeout\n" once. The host must therefore re-send commands
 *   (v1 expects ~20 Hz).
 *   Added in NALA_v0 (separate frame, does not affect the one above):
 *                           0x80 0x87 E          E=1 command echo on, E=0 off
 *                           -> MCU replies "echo on\n" / "echo off\n"
 *   MCU -> host (telemetry, once per TELEMETRY_EVERY_N_TICKS periods = 20 Hz in v1.1.0):
 *       "c <n1> <n2> <n3> <n4>\n"   cumulative encoder counts of M1..M4 since power-up
 *                                  (int32, + = the wheel drove the robot forward)
 *   Boot text: "a\n" (ADC init) and "test\n".
 *
 * WIRING / MAPPING : see config.h (motor <-> encoder association, signs, pins).
 * ---------------------------------------------------------------------------
 */

#include "config.h"

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "ADC.h"
#include "Usart.h"
#include "encoder.h"
#include "motor_functions.h"
#include "pwm.h"
#include "timer.h"

#define NUM_MOTORS 4

/* ------------------------------------------------------------------------- */
/* Motor <-> encoder association, built from the wiring map in config.h       */
/* ------------------------------------------------------------------------- */
/* motor_encoder[m] = index (0..3 = E1..E4) of the encoder that measures motor M(m+1). */
static const uint8_t motor_encoder[NUM_MOTORS] = {
	MOTOR1_ENCODER - 1, MOTOR2_ENCODER - 1, MOTOR3_ENCODER - 1, MOTOR4_ENCODER - 1
};
/* Sign applied to that encoder's counts so that + means "wheel drives forward". */
static const int8_t motor_enc_sign[NUM_MOTORS] = {
	MOTOR1_ENC_SIGN, MOTOR2_ENC_SIGN, MOTOR3_ENC_SIGN, MOTOR4_ENC_SIGN
};

/* ------------------------------------------------------------------------- */
/* Controller state (main-loop only)                                          */
/* ------------------------------------------------------------------------- */
static float M_w[NUM_MOTORS];            /* target wheel speed   [rad/s] */
static float M_w_measured[NUM_MOTORS];   /* measured wheel speed [rad/s] */
static float M_pwm[NUM_MOTORS];          /* motor power command  [% duty, -100..100] */
static float M_i[NUM_MOTORS];            /* integral part of the speed controller [% duty] */
static int32_t M_count[NUM_MOTORS];      /* cumulative encoder counts since power-up (+ = forward), sent to the host */

/* Command timeout bookkeeping (main-loop only, see CMD_TIMEOUT_MS in config.h). */
static uint8_t cmd_age_ticks;    /* control periods since the last velocity frame (saturates at 255) */
static uint8_t cmd_active;       /* 1 = the last command was non-zero, i.e. the robot may be moving */

/* ------------------------------------------------------------------------- */
/* Control time base: Timer3 compare interrupt                                */
/* ------------------------------------------------------------------------- */
static volatile int32_t tick_delta[ENC_COUNT];   /* encoder counts accumulated since last consumed */
static volatile uint8_t tick_periods;            /* control periods elapsed since last consumed   */
static int32_t last_raw[ENC_COUNT];              /* touched only by the Timer3 ISR */

ISR(TIMER3_COMPA_vect)
{
	int32_t raw[ENC_COUNT];
	encoder_read_raw(raw);
	for (uint8_t i = 0; i < ENC_COUNT; i++) {
		tick_delta[i] += raw[i] - last_raw[i];
		last_raw[i] = raw[i];
	}
	if (tick_periods < 255) {
		tick_periods++;
	}
}

/* ------------------------------------------------------------------------- */
/* UART receive: frame parser (same state machine as the original)            */
/* ------------------------------------------------------------------------- */
static volatile uint8_t rx_reading;                    /* 1 while collecting the payload */
static volatile uint8_t rx_count;
static volatile uint8_t rx_len;                        /* payload length of the frame being collected */
static volatile uint8_t rx_is_echo;                    /* 1 = echo-control frame, 0 = velocity frame */
static volatile uint8_t rx_prev;                       /* previous byte, for header detection */
static volatile int8_t  rx_buf[FRAME_PAYLOAD_LEN];
static volatile int8_t  cmd_val[FRAME_PAYLOAD_LEN];    /* last complete command: Vx, Vy, w */
static volatile uint8_t cmd_ready;                     /* 1 = cmd_val holds an unprocessed command */

/* Command echo switch (default from config.h, changed by the echo-control frame). */
static volatile uint8_t echo_enabled = ECHO_COMMAND;
static volatile uint8_t echo_ack;                      /* 0 = nothing, 1 = announce "off", 2 = announce "on" */

ISR(USART0_RX_vect)
{
	uint8_t b = UDR0;

	if (rx_reading) {
		rx_buf[rx_count++] = (int8_t)b;
		if (rx_count >= rx_len) {
			if (rx_is_echo) {
				/* Echo-control frame: 0x80 0x87 E   (E = 1 on, 0 off, anything else ignored) */
				if (rx_buf[0] == 0 || rx_buf[0] == 1) {
					echo_enabled = (uint8_t)rx_buf[0];
					echo_ack     = (uint8_t)(1 + rx_buf[0]);
				}
			} else {
				/* Velocity frame: 0x80 0x86 Vx Vy w */
				for (uint8_t i = 0; i < FRAME_PAYLOAD_LEN; i++) {
					cmd_val[i] = rx_buf[i];
				}
				cmd_ready = 1;       /* a newer frame simply replaces an unprocessed one */
			}
			rx_reading = 0;
			rx_count   = 0;
			rx_prev    = 0;          /* reset the header detection */
		}
		return;
	}

	/* Start flag = 0x80 0x86 (velocity) or 0x80 0x87 (echo control) */
	if (rx_prev == FRAME_HDR1 && (b == FRAME_HDR2 || b == FRAME_HDR2_ECHO)) {
		rx_reading = 1;
		rx_count   = 0;
		rx_is_echo = (b == FRAME_HDR2_ECHO);
		rx_len     = rx_is_echo ? FRAME_ECHO_LEN : FRAME_PAYLOAD_LEN;
	}
	rx_prev = b;
}

/* ------------------------------------------------------------------------- */
/* Kinematics                                                                 */
/* ------------------------------------------------------------------------- */
/*
 * Mecanum inverse kinematics. Vx, Vy in m/s, w in rad/s. Result: target wheel
 * speeds M_w[] in rad/s (wheel speed = rim speed / radius) and the immediate
 * motor power (feed-forward + the integral already accumulated).
 * Rim speed of each wheel = Vx -/+ Vy -/+ (W+H)*w.
 *
 * The integral M_i[] is cleared when a wheel's target becomes zero or changes sign;
 * otherwise it is kept, so a stream of similar commands (e.g. 20 per second) does
 * not throw away what the controller has learned.
 */
static void calc_angular_speed(float Vx, float Vy, float w)
{
	const float k = 1.0f / WHEEL_RADIUS_M;
	float target[NUM_MOTORS];

	target[0] = k * (Vx - Vy - ROT_ARM_M * w);   /* M1 */
	target[1] = k * (Vx + Vy - ROT_ARM_M * w);   /* M2 */
	target[2] = k * (Vx - Vy + ROT_ARM_M * w);   /* M3 */
	target[3] = k * (Vx + Vy + ROT_ARM_M * w);   /* M4 */

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		const uint8_t stopped = (target[m] < W_ZERO_EPS && target[m] > -W_ZERO_EPS);
		if (stopped) {
			target[m] = 0.0f;
			M_i[m]    = 0.0f;
			M_pwm[m]  = 0.0f;
		} else {
			if ((target[m] > 0.0f) != (M_w[m] > 0.0f)) {
				M_i[m] = 0.0f;                    /* direction change (or start from rest) */
			}
			M_pwm[m] = PWM_PER_RAD_S * target[m] + M_i[m];
		}
		M_w[m] = target[m];
	}
}

/* Clamp the power commands and send them to the motors. */
static void set_motor_speed(void)
{
	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		if (M_pwm[m] >  PWM_LIMIT) M_pwm[m] =  PWM_LIMIT;
		if (M_pwm[m] < -PWM_LIMIT) M_pwm[m] = -PWM_LIMIT;
	}
	set_motor_power(M_pwm[0], M_pwm[1], M_pwm[2], M_pwm[3]);
}

/* ------------------------------------------------------------------------- */
/* Speed controller: positional PI + feed-forward + anti-windup (new in v1)   */
/* ------------------------------------------------------------------------- */
/*
 * e   = target - measured                         [rad/s]  (textbook sign)
 * pwm = PWM_PER_RAD_S * target + PI_KP*e + I      [% duty]
 * I  += PI_KI * e * dt, clamped to +-PI_I_LIMIT, and NOT integrated while the
 *       output is saturated (unless the error pulls it back out).
 * A wheel with target 0 gets output 0 and I = 0.
 * dt is the real elapsed time of the measurement in seconds.
 *
 * The v0 controller added a PID output to the power every period (pwm -= u), which
 * made the integral term a double integrator; together with truncating u to an int
 * and keeping the integral after a stop it rang for a long time. See tests/pid_sim.py.
 */
static void speed_control(uint8_t m, float dt)
{
	if (M_w[m] == 0.0f) {
		M_i[m]   = 0.0f;
		M_pwm[m] = 0.0f;
		return;
	}

	const float e  = M_w[m] - M_w_measured[m];
	const float ff = PWM_PER_RAD_S * M_w[m];
	const float u  = ff + PI_KP * e + M_i[m];

	/* Anti-windup: integrate unless the output is saturated AND the error would push it further. */
	if ((u < PWM_LIMIT && u > -PWM_LIMIT) || ((u > 0.0f) != (e > 0.0f))) {
		M_i[m] += PI_KI * e * dt;
		if (M_i[m] >  PI_I_LIMIT) M_i[m] =  PI_I_LIMIT;
		if (M_i[m] < -PI_I_LIMIT) M_i[m] = -PI_I_LIMIT;
	}

	M_pwm[m] = ff + PI_KP * e + M_i[m];
}

/* ------------------------------------------------------------------------- */
/* Main-loop work                                                             */
/* ------------------------------------------------------------------------- */
/* A command frame arrived: update the targets (feed-forward) and apply it now. */
static void handle_command(void)
{
	int8_t v[FRAME_PAYLOAD_LEN];

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for (uint8_t i = 0; i < FRAME_PAYLOAD_LEN; i++) {
			v[i] = cmd_val[i];
		}
		cmd_ready = 0;
	}

	float Vx = v[0] / CMD_SCALE;
	float Vy = v[1] / CMD_SCALE;
	float w  = v[2] / CMD_SCALE;
	calc_angular_speed(Vx, Vy, w);

	/* Any frame (even a zero one) proves the host is alive: restart the timeout. */
	cmd_age_ticks = 0;
	cmd_active    = (v[0] != 0 || v[1] != 0 || v[2] != 0);

	if (echo_enabled) {
		char line[48];
		int n = snprintf(line, sizeof(line), "%f %f %f\n", Vx, Vy, w);
		if (n > 0 && n < (int)sizeof(line)) {
			usart_try_send_buf(line, (uint8_t)n);
		}
	}

	/* Output the feed-forward immediately instead of waiting for the next control period. */
	set_motor_speed();
}

/* A control period elapsed: measure, run the speed controllers, drive the motors, report. */
static void control_step(void)
{
	int32_t delta[ENC_COUNT];
	uint8_t periods;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for (uint8_t i = 0; i < ENC_COUNT; i++) {
			delta[i] = tick_delta[i];
			tick_delta[i] = 0;
		}
		periods = tick_periods;
		tick_periods = 0;
	}
	if (periods == 0) {
		return;
	}

	/* Exact elapsed time (normally exactly one period; more if the loop was late). */
	const float dt = (float)periods * CONTROL_PERIOD_S;

#if (CMD_TIMEOUT_MS > 0)
	/* Command timeout: no velocity frame for too long while moving -> stop. */
	if (cmd_active) {
		uint16_t age = (uint16_t)cmd_age_ticks + periods;
		cmd_age_ticks = (age > 255u) ? 255u : (uint8_t)age;
		if (cmd_age_ticks > CMD_TIMEOUT_TICKS) {
			calc_angular_speed(0, 0, 0);          /* targets = 0, feed-forward PWM = 0 */
			cmd_active = 0;                       /* report only once */
			usart_try_send_buf("cmd timeout\n", 12);
		}
	}
#endif

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		int32_t counts = delta[motor_encoder[m]] * motor_enc_sign[m];
		M_w_measured[m] = ((float)counts / ENC_COUNTS_PER_REV) / dt * TWO_PI_F;   /* rad/s */
		M_count[m] += counts;
	}

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		speed_control(m, dt);
	}
	set_motor_speed();

	/* Telemetry (20 Hz): cumulative encoder counts; dropped (never blocking) if the TX buffer is full. */
	static uint8_t tel_count;
	if (++tel_count >= TELEMETRY_EVERY_N_TICKS) {
		tel_count = 0;
		char line[80];
		int n = snprintf(line, sizeof(line), TELEMETRY_FMT,
		                 (long)M_count[0], (long)M_count[1], (long)M_count[2], (long)M_count[3]);
		if (n > 0 && n < (int)sizeof(line)) {
			usart_try_send_buf(line, (uint8_t)n);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* Entry point                                                                */
/* ------------------------------------------------------------------------- */
int main(void)
{
	/* Motor outputs: PWM pins (PD6, PD5, PB1, PB2) and direction pins (PORTE), all low. */
	init_motor_pins();
	PORTD &= ~(_BV(PD6) | _BV(PD5));
	PORTB &= ~(_BV(PB1) | _BV(PB2));
	DDRE  |= _BV(MOTOR1_DIR_BIT) | _BV(MOTOR2_DIR_BIT) | _BV(MOTOR3_DIR_BIT) | _BV(MOTOR4_DIR_BIT);
	PORTE &= ~(_BV(MOTOR1_DIR_BIT) | _BV(MOTOR2_DIR_BIT) | _BV(MOTOR3_DIR_BIT) | _BV(MOTOR4_DIR_BIT));

	init_motor_timers();
	set_motor_power(0, 0, 0, 0);

	usart_enable(UART_BAUD);
	adc_init(0);                       /* 5 V reference; prints "a\n". The ADC itself is unused. */
	usart_send_str("test\n");

	calc_angular_speed(0, 0, 0);       /* start with all targets = 0 */
	set_motor_speed();

	encoder_init();
	control_timer_init();
	sei();

	for (;;) {
		if (echo_ack) {                /* confirm an echo-control frame (sent from here, not from the ISR) */
			uint8_t ack;
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				ack = echo_ack;
				echo_ack = 0;
			}
			if (ack == 2) {
				usart_try_send_buf("echo on\n", 8);
			} else {
				usart_try_send_buf("echo off\n", 9);
			}
		}
		if (cmd_ready) {
			handle_command();
		}
		if (tick_periods) {
			control_step();
		}
	}
}
