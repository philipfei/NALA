/*
 * main.c  --  NALA motor driver firmware
 *
 * Project  : NALA (ATmega328PB, 4 mecanum wheels, 4 quadrature encoders)
 * Author   : HY / Claude (NALA v1 revision)
 * Version  : v2.3.3
 * Date     : 2026-10-19
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
 * v2.2.0: setting mode: the controller parameters (PI gains, integral limit, target ramp,
 *         feed-forward table) are read and written over the UART while stopped and saved
 *         in the EEPROM. Target ramp (accel_limit). A non-zero command is applied at the
 *         next control step (no P-less output between steps).
 * v2.3.0: one feed-forward PWM row per motor (shared speed points): the motors differ.
 * v2.3.1: new defaults in config.h (values tuned on the floor). No code change.
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
 *                  or parameter frame (one slot each) and sets a flag.
 *   - UART TX    : ring buffer drained by the UDRE interrupt             (USART.c)
 *   - main loop  : (1) check (CRC) and apply a newly received command,
 *                  (2) check (CRC) and answer a parameter request (setting mode),
 *                  (3) when a control period has elapsed: compute speeds, run the
 *                      PI controllers, set the motor output, send the state frame.
 *
 * CONCURRENCY RULE: param[], the targets and the controller state are used ONLY in the
 *   main loop (handle_command, handle_param, control_step run one after the other), never
 *   in an interrupt. So a float is never read half-written, and the control is paused
 *   while a parameter changes. If the controller is ever moved into an interrupt, wrap
 *   every parameter write in ATOMIC_BLOCK.
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
 *   Pi -> MCU: 0xA5 0x03 op idx value (float32 LE) crc8          parameter request
 *   MCU -> Pi: 0xA5 0x04 op idx status value (float32 LE) crc8   parameter reply
 *   Safety: no valid command for CMD_TIMEOUT_MS (200 ms) while moving -> stop.
 *
 * WIRING / MAPPING : see config.h (motor <-> encoder association, signs, pins).
 * ---------------------------------------------------------------------------
 */

#include "config.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/crc16.h>

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
static float M_goal[NUM_MOTORS];         /* commanded wheel speed [rad/s] (from the kinematics) */
static float M_w[NUM_MOTORS];            /* target wheel speed used by the controller [rad/s] (ramped toward M_goal) */
static float M_w_measured[NUM_MOTORS];   /* measured wheel speed [rad/s] */
static float M_pwm[NUM_MOTORS];          /* motor power command  [% duty, -100..100] */
static float M_i[NUM_MOTORS];            /* integral part of the speed controller [% duty] */
static int32_t M_count[NUM_MOTORS];      /* cumulative encoder counts since power-up (+ = forward), sent to the Pi */
static uint8_t seq;                      /* control period counter (wraps), sent to the Pi */

/* Command timeout bookkeeping (main-loop only, see CMD_TIMEOUT_MS in config.h). */
static uint8_t cmd_age_ticks;    /* control periods since the last valid command (saturates at 255) */
static uint8_t cmd_fresh;        /* 1 = a valid command was handled since the last control step */
static uint8_t cmd_ticks_before; /* control periods already pending (not yet consumed) when it was handled */
static uint8_t cmd_active;       /* 1 = the last command was non-zero, i.e. the robot may be moving */
static uint8_t timed_out;        /* 1 = stopped by the timeout, until the next valid command */
static uint8_t boot = 1;         /* 1 = no valid command since power-up (state flags bit0) */

/* ------------------------------------------------------------------------- */
/* Run-time parameters (setting mode, v2.2.0)                                 */
/* ------------------------------------------------------------------------- */
/* Index of each parameter in param[] (same order as mcu_protocol.PARAM_NAMES on the Pi). */
enum {
	P_KP, P_KI, P_I_LIMIT, P_ACCEL,
	P_FF_SPEED,                               /* ff_speed0..9 [rad/s], shared by the motors */
	P_FF_PWM = P_FF_SPEED + FF_TABLE_N,       /* ff_m1_pwm0..9, then m2, m3, m4 [% duty] */
	NUM_PARAMS = P_FF_PWM + NUM_MOTORS * FF_TABLE_N
};

/* Parameter request ops and reply status (protocol block in config.h). */
enum { OP_GET, OP_SET, OP_SAVE, OP_DEFAULTS, OP_RELOAD };
enum { ST_OK, ST_BAD_INDEX, ST_OUT_OF_RANGE, ST_BUSY, ST_BAD_TABLE, ST_BAD_OP, ST_NO_EEPROM };

/* State of the feed-forward table after edits (it is checked only when it is needed). */
enum { FF_OK, FF_UNCHECKED, FF_INVALID };

static float param[NUM_PARAMS];          /* the values in use (main loop only, see CONCURRENCY RULE) */
static uint8_t ff_state = FF_OK;
static uint8_t params_in_eeprom;         /* 1 = param[] equals the EEPROM block (state flags bit2) */

static const float ff_speed_default[FF_TABLE_N] = FF_SPEED_RAD_S;   /* rad/s, increasing */
static const float ff_pwm_default[NUM_MOTORS][FF_TABLE_N] = {       /* % duty, one row per motor */
	FF_M1_PWM_PCT, FF_M2_PWM_PCT, FF_M3_PWM_PCT, FF_M4_PWM_PCT
};

/* EEPROM block (CRC-16 over version and data, see config.h). */
typedef struct {
	uint8_t  version;
	float    value[NUM_PARAMS];
	uint16_t crc;
} param_block_t;

static param_block_t EEMEM ee_params;

static void load_defaults(void)
{
	param[P_KP]      = PI_KP;
	param[P_KI]      = PI_KI;
	param[P_I_LIMIT] = PI_I_LIMIT;
	param[P_ACCEL]   = ACCEL_LIMIT;
	for (uint8_t i = 0; i < FF_TABLE_N; i++) {
		param[P_FF_SPEED + i] = ff_speed_default[i];
		for (uint8_t m = 0; m < NUM_MOTORS; m++) {
			param[P_FF_PWM + m * FF_TABLE_N + i] = ff_pwm_default[m][i];
		}
	}
	ff_state = FF_OK;
}

/* Largest allowed value of a parameter (all minimums are 0). */
static float param_max(uint8_t idx)
{
	if (idx >= P_FF_PWM)   return PARAM_FF_PWM_MAX;
	if (idx >= P_FF_SPEED) return PARAM_FF_SPEED_MAX;
	switch (idx) {
	case P_KP:      return PARAM_KP_MAX;
	case P_KI:      return PARAM_KI_MAX;
	case P_I_LIMIT: return PARAM_I_LIMIT_MAX;
	default:        return PARAM_ACCEL_MAX;
	}
}

/* 1 if v is a finite number within 0 .. param_max(idx). NaN and +-Inf are rejected. */
static uint8_t param_in_range(uint8_t idx, float v)
{
	return isfinite(v) && v >= 0.0f && v <= param_max(idx);
}

/* 1 if the feed-forward table in t[] is usable: speeds start at 0 and strictly increase. */
static uint8_t ff_table_valid(const float *t)
{
	if (t[P_FF_SPEED] != 0.0f) {
		return 0;
	}
	for (uint8_t i = 1; i < FF_TABLE_N; i++) {
		if (!(t[P_FF_SPEED + i] > t[P_FF_SPEED + i - 1])) {
			return 0;
		}
	}
	return 1;
}

static uint16_t block_crc(const param_block_t *b)
{
	const uint8_t *p = (const uint8_t *)b;
	uint16_t crc = 0;
	for (uint8_t i = 0; i < offsetof(param_block_t, crc); i++) {
		crc = _crc_xmodem_update(crc, p[i]);
	}
	return crc;
}

/* Load param[] from the EEPROM. Returns 0 (param[] unchanged) if the block is not valid. */
static uint8_t load_eeprom(void)
{
	param_block_t b;
	eeprom_read_block(&b, &ee_params, sizeof b);
	if (b.version != EEPROM_LAYOUT_VERSION || b.crc != block_crc(&b) || !ff_table_valid(b.value)) {
		return 0;
	}
	for (uint8_t i = 0; i < NUM_PARAMS; i++) {
		if (!param_in_range(i, b.value[i])) {
			return 0;
		}
	}
	memcpy(param, b.value, sizeof param);
	ff_state = FF_OK;
	return 1;
}

/* Write param[] to the EEPROM. Blocks the main loop up to ~0.35 s (3.4 ms per changed byte);
 * the interrupts keep running (encoders, Timer3 tick, UART RX into its slots, UART TX). */
static void save_eeprom(void)
{
	param_block_t b;
	b.version = EEPROM_LAYOUT_VERSION;
	memcpy(b.value, param, sizeof param);
	b.crc = block_crc(&b);
	eeprom_update_block(&b, &ee_params, sizeof b);
}

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
 * Receive state machine for the command frame    0xA5 0x01 <6 data bytes> <crc>
 *                     and the parameter request  0xA5 0x03 <6 data bytes> <crc>:
 *   RX_IDLE       : only here a 0xA5 starts a frame (-> RX_HEADER_ACK). Other bytes are dropped.
 *   RX_HEADER_ACK : 0x01 or 0x03 -> remember the id, RX_PAYLOAD. 0xA5 -> stay here (a fresh
 *                   header). Other -> RX_IDLE.
 *   RX_PAYLOAD    : store the 6 data bytes and the crc. A 0xA5 in here is data: it never
 *                   restarts the frame.
 *   complete      : copy the bytes to the slot of that id (cmd_frame or param_frame), set its
 *                   ready flag, back to RX_IDLE. Two slots: the stream of drive commands can
 *                   never overwrite a parameter request that waits (e.g. during an EEPROM save).
 * The interrupt does no checking: the CRC is checked in the main loop (handle_command, handle_param).
 */
enum { RX_IDLE, RX_HEADER_ACK, RX_PAYLOAD };

static volatile uint8_t rx_state = RX_IDLE;
static volatile uint8_t rx_id;                           /* id of the frame being received */
static volatile uint8_t rx_count;
static volatile uint8_t rx_buf[FRAME_CMD_PAYLOAD];       /* data + crc of the frame being received */
static volatile uint8_t cmd_frame[FRAME_CMD_PAYLOAD];    /* data + crc of the last complete command */
static volatile uint8_t cmd_ready;                       /* 1 = cmd_frame holds an unprocessed frame */
static volatile uint8_t param_frame[FRAME_CMD_PAYLOAD];  /* data + crc of the last complete parameter request */
static volatile uint8_t param_ready;                     /* 1 = param_frame holds an unprocessed frame */

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
		if (b == FRAME_ID_CMD || b == FRAME_ID_PARAM) {
			rx_id    = b;
			rx_count = 0;
			rx_state = RX_PAYLOAD;
		} else if (b != FRAME_SYNC) {
			rx_state = RX_IDLE;
		}
		break;
	default:                            /* RX_PAYLOAD */
		rx_buf[rx_count++] = b;
		if (rx_count >= FRAME_CMD_PAYLOAD) {             /* complete */
			if (rx_id == FRAME_ID_CMD) {
				for (uint8_t i = 0; i < FRAME_CMD_PAYLOAD; i++) {
					cmd_frame[i] = rx_buf[i];
				}
				cmd_ready = 1;          /* a newer command simply replaces an unprocessed one */
			} else {
				for (uint8_t i = 0; i < FRAME_CMD_PAYLOAD; i++) {
					param_frame[i] = rx_buf[i];
				}
				param_ready = 1;        /* the Pi sends one request at a time */
			}
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
	f[3] = (uint8_t)(boot | (timed_out << 1) | (params_in_eeprom << 2) | ((ff_state == FF_INVALID) << 3));
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
/* Feed-forward: measured motor curve (param[] ff_speed / ff_pwm)             */
/* ------------------------------------------------------------------------- */
/* PWM [%] for motor m at a wheel target speed [rad/s]: linear interpolation in the shared speed
 * points and the PWM row of that motor, same sign as the target.
 * Segments whose speeds do not increase are skipped (never a division by zero, even while
 * the table is being edited; driving with an invalid table is blocked in handle_command). */
static float feed_forward(uint8_t m, float target)
{
	const float *ff_speed = &param[P_FF_SPEED];
	const float *ff_pwm   = &param[P_FF_PWM + m * FF_TABLE_N];
	const float s = (target < 0.0f) ? -target : target;
	float pwm = ff_pwm[FF_TABLE_N - 1];                     /* faster than the table: last point */
	for (uint8_t i = 1; i < FF_TABLE_N; i++) {
		if (!(ff_speed[i] > ff_speed[i - 1])) {
			continue;
		}
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
 * Mecanum inverse kinematics. Vx, Vy in m/s, w in rad/s. Result: the commanded wheel
 * speeds M_goal[] in rad/s (wheel speed = rim speed / radius).
 * Rim speed of each wheel = Vx -/+ Vy -/+ (W+H)*w.
 *
 * A wheel whose goal is zero stops at once (target, integral and output 0: it coasts).
 * A non-zero goal is applied by the next control step (ramp_target, <= 50 ms later), so
 * every output comes from the full controller (v2.1.0 wrote FF + I without the P term here).
 */
static void calc_angular_speed(float Vx, float Vy, float w)
{
	const float k = 1.0f / WHEEL_RADIUS_M;

	M_goal[0] = k * (Vx - Vy - ROT_ARM_M * w);   /* M1 */
	M_goal[1] = k * (Vx + Vy - ROT_ARM_M * w);   /* M2 */
	M_goal[2] = k * (Vx - Vy + ROT_ARM_M * w);   /* M3 */
	M_goal[3] = k * (Vx + Vy + ROT_ARM_M * w);   /* M4 */

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		if (M_goal[m] < W_ZERO_EPS && M_goal[m] > -W_ZERO_EPS) {
			M_goal[m] = 0.0f;
			M_w[m]    = 0.0f;
			M_i[m]    = 0.0f;
			M_pwm[m]  = 0.0f;
		}
	}
}

/*
 * Move the controller target M_w[m] toward the goal: at most accel_limit * dt per step
 * (accel_limit 0 = jump to the goal). The integral is cleared when the wheel starts from 0
 * or changes direction; otherwise it is kept, so a stream of similar commands (20 per
 * second) does not throw away what the controller has learned.
 */
static void ramp_target(uint8_t m, float dt)
{
	const float goal = M_goal[m];
	const float prev = M_w[m];
	float next = goal;

	if (goal != 0.0f && param[P_ACCEL] > 0.0f) {
		const float step = param[P_ACCEL] * dt;
		if (next > prev + step) next = prev + step;
		if (next < prev - step) next = prev - step;
	}
	if (prev == 0.0f || (next > 0.0f) != (prev > 0.0f)) {
		M_i[m] = 0.0f;
	}
	M_w[m] = next;
}

/* 1 if no wheel is commanded to move (the setting mode is open only then). */
static uint8_t robot_stopped(void)
{
	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		if (M_goal[m] != 0.0f || M_w[m] != 0.0f) {
			return 0;
		}
	}
	return 1;
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
 * pwm = FF(target) + kp*e + I                     [% duty]
 * I  += ki * e * dt, clamped to +-i_limit, and NOT integrated while the
 *       output is saturated (unless the error pulls it back out).
 * kp, ki, i_limit and the FF table are run-time parameters (param[], setting mode).
 * A wheel with target 0 gets output 0 and I = 0.
 * dt is the control time step: the elapsed time, but at most one period (after a long
 * pause, e.g. an EEPROM save, the integral and the ramp do not jump).
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

	const float kp = param[P_KP];
	const float i_limit = param[P_I_LIMIT];
	const float e  = M_w[m] - M_w_measured[m];
	const float ff = feed_forward(m, M_w[m]);
	const float u  = ff + kp * e + M_i[m];

	/* Anti-windup: integrate unless the output is saturated AND the error would push it further. */
	if ((u < PWM_LIMIT && u > -PWM_LIMIT) || ((u > 0.0f) != (e > 0.0f))) {
		M_i[m] += param[P_KI] * e * dt;
		if (M_i[m] >  i_limit) M_i[m] =  i_limit;
		if (M_i[m] < -i_limit) M_i[m] = -i_limit;
	}

	M_pwm[m] = ff + kp * e + M_i[m];
}

/* ------------------------------------------------------------------------- */
/* Main-loop work                                                             */
/* ------------------------------------------------------------------------- */
/* A command frame arrived: check its CRC and update the wheel goals. A stop is applied now,
 * a non-zero goal by the next control step. With an invalid FF table a non-zero command is
 * treated as a stop (state flags bit3). */
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
	uint8_t moving = (v[0] != 0 || v[1] != 0 || v[2] != 0);
	if (moving && ff_state == FF_UNCHECKED) {
		ff_state = ff_table_valid(param) ? FF_OK : FF_INVALID;   /* checked once after an edit */
	}
	if (ff_state == FF_INVALID) {
		v[0] = v[1] = v[2] = 0;
		moving = 0;
	}
	calc_angular_speed(v[0] / CMD_SCALE, v[1] / CMD_SCALE, v[2] / CMD_SCALE);

	/* Any valid command (even a zero one) proves the Pi is alive: restart the timeout. */
	cmd_age_ticks = 0;
	cmd_fresh     = 1;
	cmd_ticks_before = tick_periods;   /* periods that passed before this command (e.g. an EEPROM save) */
	cmd_active    = moving;
	timed_out     = 0;
	boot          = 0;

	/* Wheels with a zero goal stop now; the others keep their output until the next control step. */
	set_motor_speed();
}

/* Queue the parameter reply  0xA5 0x04 op idx status value crc  (dropped if the TX buffer is full:
 * the Pi then times out and asks again). */
static void send_param_reply(uint8_t op, uint8_t idx, uint8_t status)
{
	uint8_t f[FRAME_REPLY_LEN];
	const float value = (idx < NUM_PARAMS) ? param[idx] : 0.0f;

	f[0] = FRAME_SYNC;
	f[1] = FRAME_ID_PARAM_REPLY;
	f[2] = op;
	f[3] = idx;
	f[4] = status;
	memcpy(&f[5], &value, 4);                    /* AVR float = IEEE 754 single, little endian */
	f[FRAME_REPLY_LEN - 1] = crc8(&f[1], FRAME_REPLY_LEN - 2);
	usart_try_send_buf((const char *)f, FRAME_REPLY_LEN);
}

/* A parameter request arrived (setting mode): check its CRC, do the op, answer. Every op except
 * get needs the robot stopped. Runs in the main loop (CONCURRENCY RULE at the top). */
static void handle_param(void)
{
	uint8_t b[1 + FRAME_CMD_PAYLOAD];  /* id, op, idx, 4 value bytes, crc */

	b[0] = FRAME_ID_PARAM;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for (uint8_t i = 0; i < FRAME_CMD_PAYLOAD; i++) {
			b[1 + i] = param_frame[i];
		}
		param_ready = 0;
	}
	if (crc8(b, FRAME_CMD_PAYLOAD) != b[FRAME_CMD_PAYLOAD]) {
		return;                        /* broken frame: no reply, the Pi asks again */
	}

	const uint8_t op  = b[1];
	const uint8_t idx = b[2];
	float value;
	memcpy(&value, &b[3], 4);
	uint8_t status = ST_OK;

	if (op > OP_RELOAD) {
		status = ST_BAD_OP;
	} else if ((op == OP_GET || op == OP_SET) && idx >= NUM_PARAMS) {
		status = ST_BAD_INDEX;
	} else if (op != OP_GET && !robot_stopped()) {
		status = ST_BUSY;
	} else if (op == OP_SET) {
		if (!param_in_range(idx, value)) {
			status = ST_OUT_OF_RANGE;      /* also NaN and +-Inf; the value is not changed */
		} else {
			param[idx] = value;
			params_in_eeprom = 0;
			if (idx >= P_FF_SPEED) {
				ff_state = FF_UNCHECKED;   /* checked at the next drive command or save */
			}
		}
	} else if (op == OP_SAVE) {
		if (!ff_table_valid(param)) {
			status = ST_BAD_TABLE;
		} else {
			save_eeprom();
			ff_state = FF_OK;
			params_in_eeprom = 1;
			cmd_age_ticks = 0;             /* the save blocked the loop: no timeout from that pause */
		}
	} else if (op == OP_DEFAULTS) {
		load_defaults();
		params_in_eeprom = 0;
	} else if (op == OP_RELOAD) {
		if (load_eeprom()) {
			params_in_eeprom = 1;
		} else {
			status = ST_NO_EEPROM;
		}
	}
	send_param_reply(op, idx, status);
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

	/* Exact elapsed time (normally exactly one period; more if the loop was late, e.g. during an
	 * EEPROM save). dt_meas is the time base of the speeds and of seq (the Pi odometry); the
	 * controller (integral, ramp) uses at most one period, so a long pause causes no jump. */
	const float dt_meas = (float)periods * CONTROL_PERIOD_S;
	const float dt_ctrl = (periods > 1) ? CONTROL_PERIOD_S : dt_meas;
	seq += periods;

#if (CMD_TIMEOUT_MS > 0)
	/* Command timeout: no velocity frame for too long while moving -> stop.
	 * For a command handled since the last step only the periods after it count (at least 1):
	 * a command that waited in its slot during a pause (EEPROM save) does not time out at once,
	 * and a late loop after the command is still counted in full. */
	if (cmd_active) {
		uint16_t age;
		if (cmd_fresh) {
			age = (periods > cmd_ticks_before + 1u) ? (uint16_t)(periods - cmd_ticks_before) : 1u;
		} else {
			age = (uint16_t)cmd_age_ticks + periods;
		}
		cmd_age_ticks = (age > 255u) ? 255u : (uint8_t)age;
		if (cmd_age_ticks > CMD_TIMEOUT_TICKS) {
			calc_angular_speed(0, 0, 0);          /* targets = 0, feed-forward PWM = 0 */
			cmd_active = 0;                       /* report only once */
			timed_out = 1;
		}
	}
#endif
	cmd_fresh = 0;

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		int32_t counts = delta[motor_encoder[m]] * motor_enc_sign[m];
		M_w_measured[m] = ((float)counts / ENC_COUNTS_PER_REV) / dt_meas * TWO_PI_F;   /* rad/s */
		M_count[m] += counts;
	}

	for (uint8_t m = 0; m < NUM_MOTORS; m++) {
		ramp_target(m, dt_ctrl);
		speed_control(m, dt_ctrl);
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

	load_defaults();                   /* config.h values ... */
	params_in_eeprom = load_eeprom();  /* ... replaced by a valid EEPROM block (setting mode) */

	calc_angular_speed(0, 0, 0);       /* start with all targets = 0 */
	set_motor_speed();

	encoder_init();
	control_timer_init();
	sei();

	for (;;) {
		if (cmd_ready) {
			handle_command();
		}
		if (param_ready) {
			handle_param();
		}
		if (tick_periods) {
			control_step();
		}
	}
}
