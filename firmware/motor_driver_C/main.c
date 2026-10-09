/*
 * main.c  --  NALA motor driver firmware
 *
 * Project  : NALA (ATmega328PB, 4 mecanum wheels, 4 quadrature encoders)
 * Author   : HY / Claude (NALA v1 revision)
 * Version  : v2.0.0
 * Date     : 2026-10-09
 *
 * v1 vs v0: 20 Hz control loop; speed controller replaced by a positional PI with
 * feed-forward and anti-windup (the v0 incremental PID rang); M2/M3 motor polarity
 * inverted in v1.0.0 and reverted in v1.0.1; command timeout 200 ms.
 * v1.1.0: the feedback line carries the cumulative encoder counts and is sent every
 * period (20 Hz); UART 38400 baud; feed-forward 6.5 %/(rad/s) (measured).
 * v1.1.1: feed-forward from the measured speed -> PWM table FF_* in config.h.
 * v2.0.0: the Pi link is I2C (TWI0 slave) instead of UART; commands in int16 mm/s and
 *         mrad/s; the state (cumulative counts) is read by the Pi; CRC-8 both ways.
 *
 * Original : motor_driver_C by Floris van Mourik (created 9/18/2021), with pwm/timer
 *            code by Mathan and UART code by sojim. The untouched original sources
 *            are kept in ../original_ref/.
 *
 * ---------------------------------------------------------------------------
 * OVERVIEW
 *   Receives body velocity commands (Vx, Vy, w) over I2C, converts them to four
 *   wheel speed targets with the mecanum inverse kinematics, and runs one speed
 *   controller (PI + feed-forward) per wheel on the speed measured from the
 *   encoders. Its output drives four PWM motor channels. The Pi reads the
 *   cumulative encoder counts of the four wheels over I2C (odometry on the Pi).
 *
 * STRUCTURE (no blocking loops, no software delays)
 *   - Encoders   : pin-change interrupts count edges continuously     (encoder.c)
 *   - Time base  : Timer3 compare interrupt, exactly every CONTROL_PERIOD_MS
 *                  (TIMER3_COMPA_vect below). It snapshots the encoder counts.
 *   - I2C (TWI0) : interrupt, only stores received bytes and sends bytes of a state
 *                  frame that the main loop has already built (double buffer).
 *   - UART TX    : debug text only, ring buffer drained by the UDRE interrupt (USART.c)
 *   - main loop  : (1) check (CRC) and apply a newly received command,
 *                  (2) when a control period has elapsed: compute speeds, run the
 *                      PI controllers, set the motor output, build the state frame.
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
 * PI LINK (I2C, slave address I2C_ADDRESS): see the protocol block in config.h.
 *   Pi -> MCU: 0x01 vx vy w (int16 LE; mm/s, mm/s, mrad/s) crc8
 *   MCU -> Pi: seq flags M1..M4 (int32 LE cumulative counts) crc8
 *   Safety: no valid command for CMD_TIMEOUT_MS (200 ms) while moving -> stop.
 *   UART debug text: "a\n" (ADC init), "test\n" (boot), "cmd timeout\n".
 *
 * WIRING / MAPPING : see config.h (motor <-> encoder association, signs, pins).
 * ---------------------------------------------------------------------------
 */

#include "config.h"

#include <stdint.h>
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
static int32_t M_count[NUM_MOTORS];      /* cumulative encoder counts since power-up (+ = forward), sent to the Pi */
static uint8_t seq;                      /* control period counter (wraps), sent to the Pi */

/* Command timeout bookkeeping (main-loop only, see CMD_TIMEOUT_MS in config.h). */
static uint8_t cmd_age_ticks;    /* control periods since the last valid command (saturates at 255) */
static uint8_t cmd_active;       /* 1 = the last command was non-zero, i.e. the robot may be moving */
static uint8_t timed_out;        /* 1 = stopped by the timeout, until the next valid command */

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
/* I2C slave (TWI0): the link to the Pi                                       */
/* ------------------------------------------------------------------------- */
/*
 * The ISR does no computation: the Pi 4 handles slave clock stretching badly, and
 * the TWI stretches SCL until the ISR is done, so every interrupt must be short.
 * - Write from the Pi: the ISR only stores the bytes; at STOP it sets cmd_ready.
 *   The main loop checks the CRC and applies the command (handle_command).
 * - Read by the Pi: the main loop builds the whole frame (with CRC) in the buffer
 *   that is not being sent and then switches state_ready (double buffer). At SLA+R
 *   the ISR locks the ready buffer and then only indexes into it.
 */
#define TWCR_ACK ((1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE))

static volatile uint8_t i2c_rx[I2C_CMD_LEN];             /* command bytes being received */
static volatile uint8_t i2c_rx_count;
static volatile uint8_t cmd_ready;                       /* 1 = i2c_rx holds a complete write */

static volatile uint8_t state_frame[2][I2C_STATE_LEN];   /* double buffer, built by the main loop */
static volatile uint8_t state_ready;                     /* index of the newest complete frame */
static volatile uint8_t tx_frame;                        /* index of the frame the Pi is reading */
static volatile uint8_t tx_index;                        /* next byte of tx_frame to send */
static volatile uint8_t tx_busy;                         /* 1 while the Pi reads tx_frame */
static volatile uint8_t frame_read;                      /* 1 = the Pi has read a complete frame */

ISR(TWI0_vect)
{
	switch (TWSR0 & 0xF8) {
	case 0x60:                      /* own SLA+W received, ACK sent: a command starts */
		i2c_rx_count = 0;
		cmd_ready = 0;
		break;
	case 0x80:                      /* data byte received, ACK sent */
		if (i2c_rx_count < I2C_CMD_LEN) {
			i2c_rx[i2c_rx_count++] = TWDR0;
		}
		break;
	case 0xA0:                      /* STOP or repeated START: the write is complete */
		if (i2c_rx_count == I2C_CMD_LEN) {
			cmd_ready = 1;
		}
		i2c_rx_count = 0;
		break;
	case 0xA8:                      /* own SLA+R received, ACK sent: send the ready frame */
		tx_frame = state_ready;
		tx_busy = 1;
		TWDR0 = state_frame[tx_frame][0];
		tx_index = 1;
		break;
	case 0xB8:                      /* byte sent, ACK received: send the next byte */
		TWDR0 = (tx_index < I2C_STATE_LEN) ? state_frame[tx_frame][tx_index++] : 0xFF;
		break;
	case 0xC0:                      /* byte sent, NACK received: the read is over */
	case 0xC8:                      /* last byte sent, ACK received */
		if (tx_index >= I2C_STATE_LEN) {
			frame_read = 1;
		}
		tx_index = 0;
		tx_busy = 0;
		break;
	default:                        /* bus error (0x00) or another state: release the bus */
		i2c_rx_count = 0;
		tx_index = 0;
		tx_busy = 0;
		TWCR0 = TWCR_ACK | (1 << TWSTO);
		return;
	}
	TWCR0 = TWCR_ACK;
}

static void i2c_slave_init(void)
{
	PORTC &= ~(_BV(PC4) | _BV(PC5));     /* no internal pull-ups: the level shifter has its own */
	TWAR0 = (uint8_t)(I2C_ADDRESS << 1);   /* bit0 = 0: no general call */
	TWCR0 = TWCR_ACK;
}

/* CRC-8, polynomial 0x07, initial value 0 (same as crc8() on the Pi). */
static uint8_t crc8(const volatile uint8_t *data, uint8_t len)
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
 * Build the state frame in the buffer that is not ready, then make it the ready one.
 * If the Pi is still reading that buffer (a read started before the last switch),
 * skip this period: the counts are cumulative and seq counts periods, so nothing is lost.
 */
static void build_state_frame(void)
{
	static uint8_t boot = 1;           /* flags bit0 until the Pi has read one frame */
	const uint8_t next = state_ready ^ 1;

	if (frame_read) {
		boot = 0;
	}
	if (tx_busy && tx_frame == next) {
		return;
	}
	volatile uint8_t *f = state_frame[next];
	f[0] = seq;
	f[1] = (uint8_t)(boot | (timed_out << 1));
	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		const uint32_t c = (uint32_t)M_count[m];
		f[2 + 4 * m] = (uint8_t)c;
		f[3 + 4 * m] = (uint8_t)(c >> 8);
		f[4 + 4 * m] = (uint8_t)(c >> 16);
		f[5 + 4 * m] = (uint8_t)(c >> 24);
	}
	f[I2C_STATE_LEN - 1] = crc8(f, I2C_STATE_LEN - 1);
	state_ready = next;                /* one byte: the ISR sees either the old or the new index */
}

/* UART RX is not used since v2.0.0, but usart_enable() still turns its interrupt on:
 * read and drop the byte (a PD0 pin without a cable can pick up noise). */
ISR(USART0_RX_vect)
{
	(void)UDR0;
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
/* A command arrived over I2C: check it, update the targets (feed-forward) and apply it now. */
static void handle_command(void)
{
	uint8_t b[I2C_CMD_LEN];

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for (uint8_t i = 0; i < I2C_CMD_LEN; i++) {
			b[i] = i2c_rx[i];
		}
		cmd_ready = 0;
	}
	if (b[0] != I2C_CMD_ID || crc8(b, I2C_CMD_LEN - 1) != b[I2C_CMD_LEN - 1]) {
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

	/* Output the feed-forward immediately instead of waiting for the next control period. */
	set_motor_speed();
}

/* A control period elapsed: measure, run the speed controllers, drive the motors, build the state frame. */
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
	build_state_frame();
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
	build_state_frame();               /* a valid frame (all counts 0) before the first read */

	encoder_init();
	control_timer_init();
	i2c_slave_init();
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
