/*
 * main.c  --  NALA_v0 motor driver firmware
 *
 * Project  : NALA_v0 (ATmega328PB, 4 mecanum wheels, 4 quadrature encoders)
 * Author   : HY / Claude (NALA v0 revision)
 * Version  : v0.1.0
 * Date     : 2026-10-06
 *
 * Original : motor_driver_C by Floris van Mourik (created 9/18/2021), with pwm/timer
 *            code by Mathan and UART code by sojim. The untouched original sources
 *            are kept in ../original_ref/.
 *
 * ---------------------------------------------------------------------------
 * OVERVIEW
 *   Receives body velocity commands (Vx, Vy, w) over UART, converts them to four
 *   wheel speed targets with the mecanum inverse kinematics, and runs one PID
 *   loop per wheel on the speed measured from the encoders. The PID output
 *   drives four PWM motor channels. The four measured wheel speeds are printed
 *   back over the UART.
 *
 * STRUCTURE (no blocking loops, no software delays)
 *   - Encoders   : pin-change interrupts count edges continuously     (encoder.c)
 *   - Time base  : Timer3 compare interrupt, exactly every CONTROL_PERIOD_MS
 *                  (TIMER3_COMPA_vect below). It snapshots the encoder counts.
 *   - UART RX    : interrupt, only collects the 5-byte frame and sets a flag.
 *   - UART TX    : ring buffer drained by the UDRE interrupt             (USART.c)
 *   - main loop  : (1) apply a newly received command,
 *                  (2) when a control period has elapsed: compute speeds, PID,
 *                      motor output, telemetry.
 *
 * SPEED MEASUREMENT
 *   speed [rad/s] = (counts in the period / ENC_COUNTS_PER_REV) * 2*pi / period
 *   Because the period comes from a hardware timer (and is multiplied by the
 *   number of timer ticks actually elapsed, in case the main loop is ever late),
 *   the time base is exact. The original code assumed 0.010 s per measurement
 *   window and 0.045 s per loop; the real values were larger (print and delay
 *   included), which made measured speeds and the PID I/D terms inaccurate.
 *
 * UART PROTOCOL (frame layout unchanged; scale now 0.02, see below)
 *   Host -> MCU, 9600 8N1:  0x80 0x86 Vx Vy w   (3x int8; /50 -> m/s, m/s, rad/s; 0.02 per count)
 *   NOTE: the original scale was /100. The host (Pi) must send value*50.
 *   Axes: ROS convention, Vx forward, Vy left, w counter-clockwise positive.
 *   Safety (NALA_v0): no velocity frame for CMD_TIMEOUT_MS while moving -> stop,
 *   MCU prints "cmd timeout\n" once. The host must therefore re-send commands.
 *   Added in NALA_v0 (separate frame, does not affect the one above):
 *                           0x80 0x87 E          E=1 command echo on, E=0 off
 *                           -> MCU replies "echo on\n" / "echo off\n"
 *   MCU -> host (telemetry, once per TELEMETRY_EVERY_N_TICKS periods):
 *       "<w1> \t <w2> \t <w3> \t <w4>\n"   measured wheel speeds M1..M4 in rad/s
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
static int   M_pwm[NUM_MOTORS];          /* motor power command  [-100..100] */

typedef struct {
	float old_int_error;
	float old_error;
} pid_state_t;
static pid_state_t pid_state[NUM_MOTORS];

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
 * speeds M_w[] in rad/s (wheel speed = rim speed / radius) and the initial
 * feed-forward PWM. Rim speed of each wheel = Vx -/+ Vy -/+ (W+H)*w.
 */
static void calc_angular_speed(float Vx, float Vy, float w)
{
	const float k = 1.0f / WHEEL_RADIUS_M;

	M_w[0] = k * (Vx - Vy - ROT_ARM_M * w);   /* M1 */
	M_w[1] = k * (Vx + Vy - ROT_ARM_M * w);   /* M2 */
	M_w[2] = k * (Vx - Vy + ROT_ARM_M * w);   /* M3 */
	M_w[3] = k * (Vx + Vy + ROT_ARM_M * w);   /* M4 */

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		M_pwm[m] = (int)(M_w[m] * PWM_PER_RAD_S);
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
/* PID (control law unchanged from the original)                              */
/* ------------------------------------------------------------------------- */
/*
 * The error is defined as  err = measured - target  (opposite of the textbook
 * sign), so the caller must SUBTRACT the result from the power command:
 *      M_pwm = M_pwm - u
 * dt is the real elapsed time of the measurement in seconds.
 * The result is truncated to int, as in the original.
 */
static int PID_calc(float w_measured, float dt, uint8_t motor)
{
	pid_state_t *s = &pid_state[motor];

	float err        = w_measured - M_w[motor];
	float integral   = err * dt + s->old_int_error;
	float derivative = (err - s->old_error) / dt;

	float u = K_P * err + K_I * integral + K_D * derivative;

	s->old_error     = err;
	s->old_int_error = integral;   /* keep the running integral of the error */
	return (int)u;
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

/* A control period elapsed: measure, run the PID, drive the motors, report. */
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
	}

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		int u = PID_calc(M_w_measured[m], dt, m);
		M_pwm[m] = M_pwm[m] - u;
	}
	set_motor_speed();

	/* Telemetry: same text format as the original; dropped (never blocking) if the TX buffer is full. */
	static uint8_t tel_count;
	if (++tel_count >= TELEMETRY_EVERY_N_TICKS) {
		tel_count = 0;
		char line[80];
		int n = snprintf(line, sizeof(line), "%f \t %f \t %f \t %f\n",
		                 M_w_measured[0], M_w_measured[1], M_w_measured[2], M_w_measured[3]);
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
