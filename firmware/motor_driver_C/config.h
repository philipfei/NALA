/*
 * config.h  --  NALA central configuration (robot geometry, timing, wiring map)
 *
 * Project : NALA motor driver (ATmega328PB, 4x mecanum wheels, 4x quadrature encoders)
 * Author  : HY (NALA v1 revision)
 * Version : v2.3.2
 * Date    : 2026-10-15
 *
 * v1 changes vs v0: 20 Hz control/feedback (50 ms period), speed controller
 * redesigned (positional PI + feed-forward + anti-windup, replaces the old
 * incremental PID that rang), M2/M3 motor polarity (inverted in v1.0.0, reverted in
 * v1.0.1: no motor is inverted any more), shorter command timeout (200 ms).
 * v1.1.0: feedback = cumulative encoder counts every period (20 Hz), UART 38400 baud,
 * feed-forward 6.5 %/(rad/s) from measurements on the robot.
 * v1.1.1: feed-forward from a measured speed -> PWM table (the motor curve is not linear;
 * 6.5 overshot by 30-90 %).
 * v2.0.0: Pi link over I2C (TWI0 slave). Never worked: the level shifter on the board is
 * a TXB type, which cannot do I2C. Commands in int16 mm/s and mrad/s, state frame, CRC-8.
 * v2.1.0: back to UART (PD0/PD1) with the v2.0.0 frame contents as binary frames.
 * v2.2.0: setting mode: the controller parameters below are only the DEFAULTS. At run time
 * they live in RAM (param[] in main.c), can be read and changed over the UART (parameter
 * frames, only while stopped) and saved in the EEPROM. A valid EEPROM block wins at boot.
 * New: target ramp (ACCEL_LIMIT). No D term (the speed estimate is quantised).
 * v2.3.0: one feed-forward PWM row per motor (FF_M1_PWM_PCT .. FF_M4_PWM_PCT), the speed points
 * stay shared. The motors differ: for the same speed M4 needs ~1.5x the PWM of M3.
 * v2.3.1: the defaults are the values tuned on the floor (2026-10-15), the same as saved in the EEPROM.
 * v2.3.2: half wheelbase 0.1675 m (wheelbase 335 mm, user 2026-10-16; was 0.130). Same in config/base.yaml.
 *
 * Everything that is "a number you may want to change" or "a wire you may want to
 * re-assign" lives in this file. The .c files contain no magic numbers for these.
 *
 * Derived from the original motor_driver_C project
 * (Floris van Mourik / Mathan / sojim, 2020-2022). The original sources are kept
 * unmodified in ../original_ref/.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* CPU clock (external 16 MHz crystal, verified fuses: lfuse=0xFF)            */
/* ------------------------------------------------------------------------- */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ------------------------------------------------------------------------- */
/* Robot geometry  (mm values on the right are the measured hardware)        */
/* ------------------------------------------------------------------------- */
#define WHEEL_RADIUS_M      0.040f   /* wheel radius R              : 40 mm  */
#define HALF_TRACK_M        0.160f   /* W: half left-right distance : 160 mm (track 320 mm)     */
#define HALF_WHEELBASE_M    0.1675f  /* H: half front-rear distance : 167.5 mm (wheelbase 335 mm) */

/* Lever arm used in the mecanum yaw term: (W + H). */
#define ROT_ARM_M           (HALF_TRACK_M + HALF_WHEELBASE_M)

/* ------------------------------------------------------------------------- */
/* Encoder scaling                                                            */
/* ------------------------------------------------------------------------- */
/*
 * Counts per wheel revolution. The firmware counts EVERY edge (rising and
 * falling) of channel A only and takes the direction from channel B, exactly as
 * the original code did. 1536 is the value inherited from the original code
 * (encoder lines x gear ratio). Verified on the robot (2026-10-07): a 1.17 m drive
 * measured with a tape gave 1.17 m of odometry from these counts.
 */
#define ENC_COUNTS_PER_REV  1536.0f
#define TWO_PI_F            6.28318531f

/* ------------------------------------------------------------------------- */
/* Control / measurement timing                                               */
/* ------------------------------------------------------------------------- */
/*
 * The speed-control loop is driven by a hardware timer (Timer3, CTC mode), NOT
 * by software delays, so the sampling period is exact:
 *
 *      measured speed = (encoder counts in one period) / period
 *
 * Default 50 ms = 20 Hz: control update, speed measurement and the state frame
 * for the Pi all run at this rate.
 * One encoder count is 2*pi/1536/0.05 = 0.082 rad/s at this period.
 * The PI gains below were checked by simulation for 50 ms (tests/pid_sim.py);
 * if you change the period, re-run that simulation before trusting the gains.
 */
#define CONTROL_PERIOD_MS       50UL
#define CONTROL_PERIOD_S        ((float)CONTROL_PERIOD_MS / 1000.0f)

#define CONTROL_TIMER_PRESCALER 64UL
#define CONTROL_TIMER_OCR       ((F_CPU / CONTROL_TIMER_PRESCALER) * CONTROL_PERIOD_MS / 1000UL - 1UL)
#if (CONTROL_TIMER_OCR > 65535UL)
#error "CONTROL_PERIOD_MS too long for Timer3 with this prescaler (max ~262 ms)"
#endif

/* ------------------------------------------------------------------------- */
/* Controller                                                                 */
/* ------------------------------------------------------------------------- */
/*
 * Per-wheel speed controller (v1): positional PI with feed-forward and anti-windup
 *
 *      e   = target - measured                       [rad/s]
 *      pwm = FF(target) + PI_KP * e + I              [% duty, -100..100]
 *      I  += PI_KI * e * dt                          (clamped to +-PI_I_LIMIT,
 *                                                     not integrated while saturated)
 *
 * Target 0 -> output 0 and I cleared (no hunting around zero when stopping).
 * I is also cleared when the target changes sign.
 *
 * WHY it replaced the v0 controller: v0 added the PID output to the PWM every
 * period (pwm -= u) and u already contained an integral term, i.e. a double
 * integrator; with the int truncation of u and the integral kept after a stop this
 * rings for a long time. Simulation (tests/pid_sim.py, assumed motor model) shows
 * v0 ringing in all 24 plant variants and v1 in none.
 *
 * Gains were chosen by that simulation; they are NOT tuned on the real robot.
 * No derivative term: the speed estimate is quantised (0.082 rad/s) so it only adds noise.
 */
#define PI_KP               2.0f      /* [% pwm per rad/s]  (v2.3.1, floor tuning; was 1.0) */
#define PI_KI               8.0f      /* [% pwm per rad/s per s] */
#define PI_I_LIMIT          10.0f     /* [% pwm] anti-windup clamp of the integral (v2.3.1; was 25: less windup) */

/* Target ramp (v2.2.0): each control period the target used by the controller moves toward
 * the commanded wheel speed by at most ACCEL_LIMIT * period. 0 = off (the target jumps, as before).
 * A zero command always stops at once (coast), ramp or not. */
#define ACCEL_LIMIT         0.0f      /* [rad/s^2] wheel acceleration limit, 0 = off */

/* Feed-forward FF(target): PWM [%] that makes a wheel turn at |target| rad/s, from a table
 * (linear interpolation between the points, same sign as the target).
 * The motor curve is NOT linear (measured 2026-10-07, wheels in the air, step tests;
 * on the floor the speed was only ~0.4 rad/s lower): below ~8 % the wheel does not
 * turn, 15 % -> 3.5 rad/s, 30 % -> 8 rad/s, but 100 % gives only 13 rad/s. So no single
 * factor fits (4.0 was too low at high speed, 6.5 overshot by 30-90 % at low speed).
 * The first point is the dead zone: any non-zero target gets at least 9 %.
 * If the wheels overshoot at start-up lower the PWM of that speed, if they lag raise it.
 * v2.3.0: one PWM row per motor, because the motors differ (steady-state test 2026-10-15, no
 * wheels: for 5 rad/s M1 17.3 %, M2 14.5 %, M3 13.6 %, M4 19.9 %). The speed points are shared.
 * v2.3.1 defaults = tuned 2026-10-15 on the floor (tune_motors.py, firmware/README.md section 6):
 * speed points with more points at the low end; each row = the floor load (open loop, robot straight)
 * + half of that motor's difference from the mean measured in the air. */
#define FF_TABLE_N          10
#define FF_SPEED_RAD_S      { 0.0f, 1.0f, 2.0f, 3.5f, 5.0f, 6.6f, 8.1f, 9.4f, 11.5f, 13.0f }
#define FF_M1_PWM_PCT       { 10.71f, 12.80f, 14.89f, 17.86f, 21.05f, 25.35f, 30.72f, 37.82f, 67.37f, 100.0f }
#define FF_M2_PWM_PCT       { 10.88f, 12.58f, 14.30f, 16.72f, 19.53f, 24.04f, 29.14f, 35.88f, 63.73f, 98.73f }
#define FF_M3_PWM_PCT       { 11.07f, 12.49f, 13.92f, 16.16f, 18.98f, 23.67f, 28.66f, 35.48f, 63.35f, 99.49f }
#define FF_M4_PWM_PCT       { 10.86f, 12.94f, 15.02f, 18.39f, 21.92f, 27.34f, 33.42f, 41.49f, 73.74f, 100.0f }
#define PWM_LIMIT           100.0f

/* A wheel target below this is treated as "stopped" [rad/s]. */
#define W_ZERO_EPS          0.01f

/* ------------------------------------------------------------------------- */
/* Setting mode (v2.2.0): run-time parameters, EEPROM                         */
/* ------------------------------------------------------------------------- */
/*
 * Parameter index (the Pi uses the same order, mcu_protocol.PARAM_NAMES):
 *   0 kp, 1 ki, 2 i_limit, 3 accel_limit, 4..13 ff_speed0..9,
 *   14..23 ff_m1_pwm0..9, 24..33 ff_m2_pwm0..9, 34..43 ff_m3_pwm0..9, 44..53 ff_m4_pwm0..9 (v2.3.0)
 * The defaults are the values above. A set is accepted only while the robot is stopped
 * (all wheel targets 0), only for a finite value within 0 .. the maximum below.
 */
#define PARAM_KP_MAX            50.0f     /* [% pwm per rad/s] */
#define PARAM_KI_MAX            200.0f    /* [% pwm per rad/s per s] */
#define PARAM_I_LIMIT_MAX       100.0f    /* [% pwm] */
#define PARAM_ACCEL_MAX         200.0f    /* [rad/s^2] */
#define PARAM_FF_SPEED_MAX      20.0f     /* [rad/s] */
#define PARAM_FF_PWM_MAX        100.0f    /* [% pwm] */

/*
 * EEPROM block: layout version, all parameters (float), CRC-16 (XMODEM: poly 0x1021, init 0)
 * of the version and the parameters. Change the version whenever the parameter list changes:
 * an old block is then ignored and the defaults are used.
 * The fuse hfuse 0xD1 has EESAVE set: a reflash keeps the EEPROM (and the saved parameters).
 */
#define EEPROM_LAYOUT_VERSION   2       /* v2.3.0: 54 parameters (v2.2.0: 1, 24 parameters) */

/* ------------------------------------------------------------------------- */
/* Pi link: UART0 with binary frames (v2.1.0)                                 */
/* ------------------------------------------------------------------------- */
/*
 * UART0: PD0 = RXD0, PD1 = TXD0, 38400 8N1. The Pi (3.3 V) is connected through the
 * level shifter on the board. That part is a TXB type (auto direction): fine for
 * UART, but it cannot do I2C, which is why the I2C link of v2.0.0 never worked.
 * Only binary frames are sent: no text.
 *
 * Pi -> MCU, command frame, 9 bytes:
 *      0xA5  0x01  vx_lo vx_hi  vy_lo vy_hi  w_lo w_hi  crc
 *  vx, vy [mm/s], w [mrad/s], int16 little endian. ROS axes (REP-103): vx forward,
 *  vy LEFT, w counter-clockwise positive. crc = CRC-8 (poly 0x07, init 0) of the
 *  7 bytes after the sync byte (id + data). A frame with a wrong crc is ignored.
 *  Safety: if no valid command arrives for CMD_TIMEOUT_MS the robot stops (see
 *  below), so the Pi must keep sending the command while driving (20 Hz).
 *
 * MCU -> Pi, state frame, 21 bytes, every control period (20 Hz):
 *      0xA5  0x02  seq  flags  M1[4]  M2[4]  M3[4]  M4[4]  crc
 *  seq   : control period counter (uint8, wraps). The difference of two frames is the
 *          number of control periods between them (exact time base for the speeds).
 *  flags : bit0 = no valid command since power-up (the counters started at 0),
 *          bit1 = stopped by the command timeout (until the next valid command),
 *          bit2 = the parameters in RAM are the EEPROM block (loaded or saved, unchanged since),
 *          bit3 = the feed-forward table is invalid: non-zero drive commands are ignored.
 *  Mn    : cumulative encoder counts since power-up, int32 little endian,
 *          MOTORn_ENC_SIGN applied (+ = the wheel drove the robot forward).
 *          They overflow only after about a week of driving in one direction.
 *  crc   : CRC-8 of the 19 bytes after the sync byte.
 *  21 bytes take 5.5 ms at 38400 baud: 11 % of the line at 20 Hz. A frame that does
 *  not fit into the TX ring buffer (USART.c) is dropped, never waited for.
 *
 * Setting mode (v2.2.0). Pi -> MCU, parameter request, 9 bytes (same length as a command):
 *      0xA5  0x03  op  idx  value[4]  crc
 *  op    : 0 get, 1 set (RAM), 2 save all to EEPROM, 3 load the defaults (RAM), 4 reload from EEPROM (RAM)
 *  idx   : parameter index (see "Setting mode" above); ignored by ops 2-4
 *  value : float32 little endian (only used by set)
 * MCU -> Pi, parameter reply, 10 bytes:
 *      0xA5  0x04  op  idx  status  value[4]  crc
 *  status: 0 ok, 1 bad index, 2 out of range (or not finite), 3 busy (not stopped),
 *          4 bad feed-forward table, 5 bad op, 6 no valid EEPROM block
 *  value : the parameter idx after the op (0 if idx is bad)
 *  crc   : CRC-8 of the bytes after the sync byte. The Pi sends one request at a time.
 */
#define UART_BAUD           38400   /* Normal mode: UBRR = 25, error +0.16 % at 16 MHz.
                                     * 57600 would give UBRR = 16 and +2.1 % error: do not use it. */
#define FRAME_SYNC          0xA5
#define FRAME_ID_CMD        0x01
#define FRAME_ID_STATE      0x02
#define FRAME_ID_PARAM      0x03    /* v2.2.0: Pi -> MCU parameter request (same length as a command) */
#define FRAME_ID_PARAM_REPLY 0x04   /* v2.2.0: MCU -> Pi parameter reply */
#define FRAME_CMD_PAYLOAD   7       /* bytes after sync + id in a command or parameter request: 6 data + crc */
#define FRAME_STATE_LEN     21      /* whole state frame */
#define FRAME_REPLY_LEN     10      /* whole parameter reply frame */
#define CMD_SCALE           1000.0f /* command units per m/s (or rad/s): mm/s, mrad/s */

/*
 * Command timeout (NALA_v0 addition): if the robot is moving and no valid command
 * has arrived for CMD_TIMEOUT_MS, all targets are set to zero (stop) and flags bit1
 * of the state frame is set. Any new valid command resumes normal operation. 0 disables the timeout (the last command is then held forever, as
 * in the original).
 * The check runs once per control period, so the stop happens between
 * CMD_TIMEOUT_MS and CMD_TIMEOUT_MS + CONTROL_PERIOD_MS after the last frame.
 * A zero command (stop) never times out - there is nothing to stop.
 */
#define CMD_TIMEOUT_MS      200UL   /* v1: 200 ms (v0: 500 ms); stop happens 200..250 ms after the last frame */
#define CMD_TIMEOUT_TICKS   ((CMD_TIMEOUT_MS + CONTROL_PERIOD_MS - 1UL) / CONTROL_PERIOD_MS)
#if (CMD_TIMEOUT_TICKS > 250UL)
#error "CMD_TIMEOUT_MS too long (max 250 control periods)"
#endif

/* ========================================================================= */
/*  WIRING MAP  --  edit here to change which encoder belongs to which motor  */
/* ========================================================================= */
/*
 * Motors are numbered M1..M4 in the kinematics (main.c: calc_angular_speed).
 * With the ROS convention (x forward, y left) the physical layout is
 * (forward driving and odometry verified on the robot 2026-10-07; strafe and turn
 * worked as expected in the teleop test):
 *
 *            front
 *     M1 (FL)     M4 (FR)
 *     M2 (RL)     M3 (RR)
 *            rear
 *
 * Kinematic signs: Vx -> (+ + + +), Vy -> (- + - +), w -> (- - + +) for M1..M4.
 * Verify with a pure-Vx / pure-Vy / pure-w test with the wheels lifted.
 *
 * --- 1. Motor output channels (fixed by the PWM timer pins, see pwm.c) ------
 *      Motor   PWM pin (timer output)    Direction pin
 *      M1      PD6  (OC0A)               PE0
 *      M2      PD5  (OC0B)               PE1
 *      M3      PB1  (OC1A)               PE2
 *      M4      PB2  (OC1B)               PE3
 */
#define MOTOR1_DIR_BIT      0   /* PORTE bit */
#define MOTOR2_DIR_BIT      1
#define MOTOR3_DIR_BIT      2
#define MOTOR4_DIR_BIT      3

/*
 * Motor input polarity: 1 inverts the direction pin of that motor.
 *
 * HARDWARE NOTE: the supply cables of M2 and M3 are wired with opposite polarity to
 * M1 and M4. That is how the hardware is built, so no software inversion is needed:
 * all four flags are 0 and a positive power turns all four wheels the same way.
 * Do NOT invert M2/M3 here.
 * If the supply cables of a motor are re-soldered, that wheel reverses: set its flag
 * to 1 (only then). A wiring/flag mismatch gives positive feedback in the speed loop
 * and that wheel runs away to full power.
 * (v1.0.0 had M2/M3 inverted in software; reverted in v1.0.1.)
 */
#define MOTOR1_DIR_INVERT   0
#define MOTOR2_DIR_INVERT   0
#define MOTOR3_DIR_INVERT   0
#define MOTOR4_DIR_INVERT   0

/*
 * --- 2. Encoder channels (silkscreen E1..E4): fixed by the PCB wiring --------
 *      Encoder   A pin   B pin
 *      E1        PB3     PB4
 *      E2        PD2     PD3
 *      E3        PD4     PD7
 *      E4        PC2     PC3
 * Only the A pins generate pin-change interrupts; B is sampled on demand.
 * (If you rewire an encoder to another pin, update the entries below; the
 *  PCINT vectors for ports B, C and D are already handled in encoder.c.)
 */
#define ENC1_A_PINREG   PINB
#define ENC1_A_BIT      PB3
#define ENC1_B_PINREG   PINB
#define ENC1_B_BIT      PB4
#define ENC1_DDR        DDRB
#define ENC1_A_PCMSK    PCMSK0
#define ENC1_A_PCINT    PCINT3
#define ENC1_PCIE       PCIE0

#define ENC2_A_PINREG   PIND
#define ENC2_A_BIT      PD2
#define ENC2_B_PINREG   PIND
#define ENC2_B_BIT      PD3
#define ENC2_DDR        DDRD
#define ENC2_A_PCMSK    PCMSK2
#define ENC2_A_PCINT    PCINT18
#define ENC2_PCIE       PCIE2

#define ENC3_A_PINREG   PIND
#define ENC3_A_BIT      PD4
#define ENC3_B_PINREG   PIND
#define ENC3_B_BIT      PD7
#define ENC3_DDR        DDRD
#define ENC3_A_PCMSK    PCMSK2
#define ENC3_A_PCINT    PCINT20
#define ENC3_PCIE       PCIE2

#define ENC4_A_PINREG   PINC
#define ENC4_A_BIT      PC2
#define ENC4_B_PINREG   PINC
#define ENC4_B_BIT      PC3
#define ENC4_DDR        DDRC
#define ENC4_A_PCMSK    PCMSK1
#define ENC4_A_PCINT    PCINT10
#define ENC4_PCIE       PCIE1

/*
 * --- 3. Motor <-> encoder association (the speed feedback of motor Mn is read
 *        from encoder En).  Values 1..4 = E1..E4. Each encoder must be used
 *        exactly once (checked at compile time below).
 *
 *     MOTORn_ENC_SIGN: +1 or -1. The encoder counts "positive" when channel B
 *     differs from channel A (see encoder.c). On this robot the left/right motors
 *     are mounted as mirror images, so the sign is flipped for one side so that
 *     a positive measured speed always means "wheel drives the robot forward".
 *     (Original code: M1, M2 inverted; M3, M4 not inverted.)
 */
#define MOTOR1_ENCODER      1
#define MOTOR2_ENCODER      2
#define MOTOR3_ENCODER      3
#define MOTOR4_ENCODER      4

#define MOTOR1_ENC_SIGN    (-1)
#define MOTOR2_ENC_SIGN    (-1)
#define MOTOR3_ENC_SIGN    (+1)
#define MOTOR4_ENC_SIGN    (+1)

/* Compile-time sanity checks for the mapping above. */
#if (MOTOR1_ENCODER < 1 || MOTOR1_ENCODER > 4 || MOTOR2_ENCODER < 1 || MOTOR2_ENCODER > 4 || \
     MOTOR3_ENCODER < 1 || MOTOR3_ENCODER > 4 || MOTOR4_ENCODER < 1 || MOTOR4_ENCODER > 4)
#error "MOTORn_ENCODER must be in 1..4"
#endif
#if (MOTOR1_ENCODER == MOTOR2_ENCODER || MOTOR1_ENCODER == MOTOR3_ENCODER || \
     MOTOR1_ENCODER == MOTOR4_ENCODER || MOTOR2_ENCODER == MOTOR3_ENCODER || \
     MOTOR2_ENCODER == MOTOR4_ENCODER || MOTOR3_ENCODER == MOTOR4_ENCODER)
#error "Each encoder must be assigned to exactly one motor"
#endif

#endif /* CONFIG_H_ */
