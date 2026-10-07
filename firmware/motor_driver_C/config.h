/*
 * config.h  --  NALA_v1 central configuration (robot geometry, timing, wiring map)
 *
 * Project : NALA_v1 motor driver (ATmega328PB, 4x mecanum wheels, 4x quadrature encoders)
 * Author  : HY (NALA v1 revision)
 * Version : v1.1.1
 * Date    : 2026-10-07
 *
 * v1 changes vs v0: 20 Hz control/feedback (50 ms period), speed controller
 * redesigned (positional PI + feed-forward + anti-windup, replaces the old
 * incremental PID that rang), M2/M3 motor polarity (inverted in v1.0.0, reverted in
 * v1.0.1: no motor is inverted any more), shorter command timeout (200 ms).
 * v1.1.0: feedback = cumulative encoder counts every period (20 Hz), UART 38400 baud,
 * feed-forward 6.5 %/(rad/s) from measurements on the robot.
 * v1.1.1: feed-forward from a measured speed -> PWM table (the motor curve is not linear;
 * 6.5 overshot by 30-90 %).
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
#define HALF_WHEELBASE_M    0.130f   /* H: half front-rear distance : 130 mm (wheelbase 260 mm) */

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
 * Default 50 ms = 20 Hz: control update, speed measurement and the serial
 * feedback line all run at this rate.
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

/*
 * Send the feedback line every N control periods.
 * v1.1.0: N = 1 -> 20 Hz. The line carries the CUMULATIVE encoder counts of M1..M4
 * since power-up, with MOTORn_ENC_SIGN applied (+ = the wheel drove the robot forward).
 * The host takes the difference of two lines, so a dropped line loses no distance.
 * (v1.0.x sent the speed of only every 2nd period: the counts of the other period
 * were never reported, and a dropped line lost 100 ms of motion.)
 * Line length: "c" + 4 x (space + up to 11 characters) + "\n" <= 50 bytes, about 25
 * when driving. At 38400 baud one byte takes 0.26 ms, so even the longest line
 * (13 ms) uses only 26 % of the line at 20 Hz. The command echo at 20 Hz adds ~16 %.
 * If a line does not fit in the TX buffer it is dropped, never waited for.
 * The int32 counters overflow only after about a week of driving in one direction.
 */
#define TELEMETRY_EVERY_N_TICKS 1

/* Feedback line: "c" and the cumulative encoder counts of M1..M4 (int32). */
#define TELEMETRY_FMT           "c %ld %ld %ld %ld\n"

/* Command echo: after each received velocity command, send back "Vx Vy w\n"
 * (the original always did this). This is only the power-on DEFAULT; it can be
 * switched at run time with the echo-control frame (see UART protocol below). */
#define ECHO_COMMAND            0

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
#define PI_KP               1.0f      /* [% pwm per rad/s] */
#define PI_KI               8.0f      /* [% pwm per rad/s per s] */
#define PI_I_LIMIT          25.0f     /* [% pwm] anti-windup clamp of the integral */

/* Feed-forward FF(target): PWM [%] that makes a wheel turn at |target| rad/s, from a table
 * (linear interpolation between the points, same sign as the target).
 * The motor curve is NOT linear (measured 2026-10-07, wheels in the air, step tests;
 * on the floor the speed was only ~0.4 rad/s lower): below ~8 % the wheel does not
 * turn, 15 % -> 3.5 rad/s, 30 % -> 8 rad/s, but 100 % gives only 13 rad/s. So no single
 * factor fits (4.0 was too low at high speed, 6.5 overshot by 30-90 % at low speed).
 * The first point is the dead zone: any non-zero target gets at least 9 %.
 * If the wheels overshoot at start-up lower the PWM of that speed, if they lag raise it. */
#define FF_TABLE_N          10
#define FF_SPEED_RAD_S      { 0.0f, 3.5f, 6.6f, 8.1f, 9.4f, 10.7f, 11.5f, 11.85f, 12.2f, 13.0f }
#define FF_PWM_PCT          { 9.0f, 15.0f, 21.5f, 29.5f, 37.5f, 45.5f, 54.0f, 63.0f, 72.0f, 100.0f }
#define PWM_LIMIT           100.0f

/* A wheel target below this is treated as "stopped" [rad/s]. */
#define W_ZERO_EPS          0.01f

/* ------------------------------------------------------------------------- */
/* UART protocol (frame layout as in the original; SCALE CHANGED to 0.02)     */
/* ------------------------------------------------------------------------- */
/*
 * Host -> MCU frame, 38400 8N1, 5 bytes:
 *      0x80  0x86  Vx  Vy  w
 *  Vx, Vy, w are int8_t.  Vx/50 -> m/s, Vy/50 -> m/s, w/50 -> rad/s
 *  i.e. 0.02 per count, range -2.56 .. +2.54.
 *  The ORIGINAL scale was /100 (0.01 per count): the host must now send value*50.
 *  Axes follow the ROS convention (REP-103): Vx = forward, Vy = LEFT,
 *  w = counter-clockwise positive. The host sends linear.x, linear.y, angular.z.
 *  Not verified on the robot.
 *
 *  Safety: if no velocity frame arrives for CMD_TIMEOUT_MS the robot stops
 *  (see below), so the host must keep re-sending the command while driving.
 * MCU -> host: text lines, see TELEMETRY in main.c.
 *
 * NALA_v0 ADDITION (does not touch the frame above): echo-control frame, 3 bytes:
 *      0x80  0x87  E          E = 0x01: echo ON, 0x00: echo OFF (other values ignored)
 *  The MCU answers with the text line "echo on\n" / "echo off\n".
 *  A velocity frame always starts with 0x80 0x86, so the two never collide.
 */
#define UART_BAUD           38400   /* v1.1.0 (was 9600). Normal mode: UBRR = 25, error +0.16 % at 16 MHz.
                                     * 57600 would give UBRR = 16 and +2.1 % error: do not use it. */
#define FRAME_HDR1          0x80
#define FRAME_HDR2          0x86
#define FRAME_PAYLOAD_LEN   3
#define CMD_SCALE           50.0f   /* counts per unit: 50 -> 0.02 m/s (or rad/s) per count */

#define FRAME_HDR2_ECHO     0x87    /* second header byte of the echo-control frame */
#define FRAME_ECHO_LEN      1

/*
 * Command timeout (NALA_v0 addition): if the robot is moving and no velocity
 * frame has arrived for CMD_TIMEOUT_MS, all targets are set to zero (stop) and
 * the line "cmd timeout\n" is sent once. Any new velocity frame resumes normal
 * operation. 0 disables the timeout (the last command is then held forever, as
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
