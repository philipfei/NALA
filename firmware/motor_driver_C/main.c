/*
 * main.c  --  NALA motor driver firmware
 *
 * Project  : NALA (ATmega328PB, 4 mecanum wheels, 4 quadrature encoders)
 * Author   : HY / Claude (NALA v1 revision)
 * Version  : v2.1.0
 * Date     : 2026-10-12
 *
 * v1 vs v0: 20 Hz control loop; speed controller replaced by a positional PI with
 * feed-forward and anti-windup (the v0 incremental PID rang); M2/M3 motor polarity
 * inverted in v1.0.0 and reverted in v1.0.1; command timeout 200 ms.
 * v1.1.0: the feedback line carries the cumulative encoder counts and is sent every
 * period (20 Hz); UART 38400 baud; feed-forward 6.5 %/(rad/s) (measured).
 * v1.1.1: feed-forward from the measured speed -> PWM table FF_* in config.h.
 * v2.0.0: Pi link over I2C (never worked on the robot: the board has a TXB level
 *         shifter, which cannot do I2C). Commands in int16 mm/s and mrad/s, CRC-8.
 * v2.1.0: back to UART (PD0/PD1, 38400 baud) with the v2.0.0 frame contents as binary
 *         frames: sync byte, id, data, CRC-8. No text on the UART any more.
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
 *   counts of the four wheels are sent to the Pi every period (odometry on the Pi).
 *
 * STRUCTURE (no blocking loops, no software delays)
 *   - Encoders   : pin-change interrupts count edges continuously     (encoder.c)
 *   - Time base  : Timer3 compare interrupt, exactly every CONTROL_PERIOD_MS
 *                  (TIMER3_COMPA_vect below). It snapshots the encoder counts.
 *   - UART RX    : interrupt, a small state machine that only collects a command
 *                  frame and sets a flag.
 *   - UART TX    : ring buffer drained by the UDRE interrupt             (USART.c)
 *   - main loop  : (1) check (CRC) and apply a newly received command,
 *                  (2) when a control period has elapsed: compute speeds, run the
 *                      PI controllers, set the motor output, send the state frame.
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
 * PI LINK (UART0, 38400 8N1, binary): see the protocol block in config.h.
 *   Pi -> MCU: 0xA5 0x01 vx vy w (int16 LE; mm/s, mm/s, mrad/s) crc8
 *   MCU -> Pi: 0xA5 0x02 seq flags M1..M4 (int32 LE cumulative counts) crc8, every period
 *   Safety: no valid command for CMD_TIMEOUT_MS (200 ms) while moving -> stop.
 *
 * WIRING / MAPPING : see config.h (motor <-> encoder association, signs, pins).
 * ---------------------------------------------------------------------------
 */

#include "config.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

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
static int32_t M_count[NUM_MOTORS];      /* cumulative encoder counts since power-up (+ = forward), sent to the Pi */
static uint8_t seq;                      /* control period counter (wraps), sent to the Pi */

/* Command timeout bookkeeping (main-loop only, see CMD_TIMEOUT_MS in config.h). */
static uint8_t cmd_age_ticks;    /* control periods since the last valid command (saturates at 255) */
static uint8_t cmd_active;       /* 1 = the last command was non-zero, i.e. the robot may be moving */
static uint8_t timed_out;        /* 1 = stopped by the timeout, until the next valid command */
static uint8_t boot = 1;         /* 1 = no valid command since power-up (state flags bit0) */

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
/* UART link to the Pi: binary frames                                         */
/* ------------------------------------------------------------------------- */
/*
 * Receive state machine for the command frame  0xA5 0x01 <6 data bytes> <crc>:
 *   RX_IDLE       : only here a 0xA5 starts a frame (-> RX_HEADER_ACK). Other bytes are dropped.
 *   RX_HEADER_ACK : 0x01 -> RX_PAYLOAD. 0xA5 -> stay here (a fresh header). Other -> RX_IDLE.
 *   RX_PAYLOAD    : store the 6 data bytes and the crc. A 0xA5 in here is data: it never
 *                   restarts the frame.
 *   complete      : copy the bytes to cmd_frame, set cmd_ready, back to RX_IDLE.
 * The interrupt does no checking: the CRC is checked in the main loop (handle_command).
 */
enum { RX_IDLE, RX_HEADER_ACK, RX_PAYLOAD };

static volatile uint8_t rx_state = RX_IDLE;
static volatile uint8_t rx_count;
static volatile uint8_t rx_buf[FRAME_CMD_PAYLOAD];       /* data + crc of the frame being received */
static volatile uint8_t cmd_frame[FRAME_CMD_PAYLOAD];    /* data + crc of the last complete frame */
static volatile uint8_t cmd_ready;                       /* 1 = cmd_frame holds an unprocessed frame */

ISR(USART0_RX_vect)
{
	const uint8_t b = UDR0;

	switch (rx_state) {
	case RX_IDLE:
		if (b == FRAME_SYNC) {
			rx_state = RX_HEADER_ACK;
		}
		break;
	case RX_HEADER_ACK:
		if (b == FRAME_ID_CMD) {
			rx_count = 0;
			rx_state = RX_PAYLOAD;
		} else if (b != FRAME_SYNC) {
			rx_state = RX_IDLE;
		}
		break;
	default:                            /* RX_PAYLOAD */
		rx_buf[rx_count++] = b;
		if (rx_count >= FRAME_CMD_PAYLOAD) {             /* complete */
			for (uint8_t i = 0; i < FRAME_CMD_PAYLOAD; i++) {
				cmd_frame[i] = rx_buf[i];
			}
			cmd_ready = 1;              /* a newer frame simply replaces an unprocessed one */
			rx_state = RX_IDLE;
		}
		break;
	}
}

/* CRC-8, polynomial 0x07, initial value 0 (same as crc8() on the Pi). */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * Queue the state frame  0xA5 0x02 seq flags M1..M4 crc  for the Pi (every control period).
 * If the TX buffer has no room the frame is dropped, never waited for: the counts are
 * cumulative and seq counts periods, so the Pi loses nothing.
 */
static void send_state_frame(void)
{
	uint8_t f[FRAME_STATE_LEN];

	f[0] = FRAME_SYNC;
	f[1] = FRAME_ID_STATE;
	f[2] = seq;
	f[3] = (uint8_t)(boot | (timed_out << 1));
	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		const uint32_t c = (uint32_t)M_count[m];
		f[4 + 4 * m] = (uint8_t)c;
		f[5 + 4 * m] = (uint8_t)(c >> 8);
		f[6 + 4 * m] = (uint8_t)(c >> 16);
		f[7 + 4 * m] = (uint8_t)(c >> 24);
	}
	f[FRAME_STATE_LEN - 1] = crc8(&f[1], FRAME_STATE_LEN - 2);   /* id .. last count byte */
	usart_try_send_buf((const char *)f, FRAME_STATE_LEN);
}

/* ------------------------------------------------------------------------- */
/* Feed-forward: measured motor curve (config.h FF_*)                         */
/* ------------------------------------------------------------------------- */
static const float ff_speed[FF_TABLE_N] = FF_SPEED_RAD_S;   /* rad/s, increasing */
static const float ff_pwm[FF_TABLE_N]   = FF_PWM_PCT;       /* % duty */

/* PWM [%] for a wheel target speed [rad/s]: linear interpolation in the table, same sign. */
static float feed_forward(float target)
{
	const float s = (target < 0.0f) ? -target : target;
	float pwm = ff_pwm[FF_TABLE_N - 1];                     /* faster than the table: 100 % */
	for (uint8_t i = 1; i < FF_TABLE_N; i++) {
		if (s <= ff_speed[i]) {
			pwm = ff_pwm[i - 1] + (ff_pwm[i] - ff_pwm[i - 1]) * (s - ff_speed[i - 1])
			                      / (ff_speed[i] - ff_speed[i - 1]);
			break;
		}
	}
	return (target < 0.0f) ? -pwm : pwm;
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
			M_pwm[m] = feed_forward(target[m]) + M_i[m];
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
 * pwm = FF(target) + PI_KP*e + I                  [% duty]
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
	const float ff = feed_forward(M_w[m]);
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
/* A command frame arrived: check its CRC, update the targets (feed-forward) and apply it now. */
static void handle_command(void)
{
	uint8_t b[1 + FRAME_CMD_PAYLOAD];  /* id, 6 data bytes, crc */

	b[0] = FRAME_ID_CMD;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for (uint8_t i = 0; i < FRAME_CMD_PAYLOAD; i++) {
			b[1 + i] = cmd_frame[i];
		}
		cmd_ready = 0;
	}
	if (crc8(b, FRAME_CMD_PAYLOAD) != b[FRAME_CMD_PAYLOAD]) {
		return;                        /* broken frame: ignore it (the next one comes in 50 ms) */
	}

	int16_t v[3];
	for (uint8_t i = 0; i < 3; i++) {
		v[i] = (int16_t)((uint16_t)b[1 + 2 * i] | ((uint16_t)b[2 + 2 * i] << 8));
	}
	float Vx = v[0] / CMD_SCALE;
	float Vy = v[1] / CMD_SCALE;
	float w  = v[2] / CMD_SCALE;
	calc_angular_speed(Vx, Vy, w);

	/* Any valid command (even a zero one) proves the Pi is alive: restart the timeout. */
	cmd_age_ticks = 0;
	cmd_active    = (v[0] != 0 || v[1] != 0 || v[2] != 0);
	timed_out     = 0;
	boot          = 0;

	/* Output the feed-forward immediately instead of waiting for the next control period. */
	set_motor_speed();
}

/* A control period elapsed: measure, run the speed controllers, drive the motors, send the state frame. */
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
	seq += periods;

#if (CMD_TIMEOUT_MS > 0)
	/* Command timeout: no velocity frame for too long while moving -> stop. */
	if (cmd_active) {
		uint16_t age = (uint16_t)cmd_age_ticks + periods;
		cmd_age_ticks = (age > 255u) ? 255u : (uint8_t)age;
		if (cmd_age_ticks > CMD_TIMEOUT_TICKS) {
			calc_angular_speed(0, 0, 0);          /* targets = 0, feed-forward PWM = 0 */
			cmd_active = 0;                       /* report only once */
			timed_out = 1;
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
	send_state_frame();
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

	usart_enable(UART_BAUD);           /* binary frames only: no text is printed (v2.1.0) */

	calc_angular_speed(0, 0, 0);       /* start with all targets = 0 */
	set_motor_speed();

	encoder_init();
	control_timer_init();
	sei();

	for (;;) {
		if (cmd_ready) {
			handle_command();
		}
		if (tick_periods) {
			control_step();
		}
	}
}
