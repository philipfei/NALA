/*
 * config.h  --  NALA_v1 central configuration (robot geometry, timing, wiring map)
 *
 * Project : NALA_v1 motor driver (ATmega328PB, 4x mecanum wheels, 4x quadrature encoders)
 * Author  : HY (NALA v1 revision)
 * Version : v1.0.0
 * Date    : 2026-10-07
 *
 * v1 changes vs v0: 20 Hz control/feedback (50 ms period), speed controller
 * redesigned (positional PI + feed-forward + anti-windup, replaces the old
 * incremental PID that rang), M2/M3 motor direction polarity inverted, shorter
 * command timeout (200 ms).
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
 * (encoder lines x gear ratio); it has NOT been re-verified on this hardware.
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
 * Print the 4 measured wheel speeds every N control periods.
 * v1: N = 2 -> feedback at 10 Hz while control and commands run at 20 Hz.
 * Why: the feedback line is ~50 characters (~52 ms at 9600 baud). Sent every 100 ms
 * it leaves the line about half idle and the TX buffer always drains before the
 * next line is queued - nothing piles up. Sent every 50 ms it would not fit.
 * If a line does not fit in the TX buffer it is dropped, never waited for.
 * (Turning the command echo ON at 20 Hz commands adds ~62 % line load, i.e. the total
 * exceeds the line capacity and some echo/feedback lines will be dropped - never
 * waited for. Use the echo for debugging only.)
 */
#define TELEMETRY_EVERY_N_TICKS 2

/* Feedback line: four tab-separated floats [rad/s], M1..M4 - identical to the original format. */
#define TELEMETRY_FMT           "%f \t %f \t %f \t %f\n"

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
 *      pwm = PWM_PER_RAD_S * target + PI_KP * e + I  [% duty, -100..100]
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

/* Feed-forward: PWM [%] per rad/s of target wheel speed. Assumes ~0.25 rad/s per 1 %.
 * If the wheels overshoot at start-up reduce it, if they lag behind raise it. */
#define PWM_PER_RAD_S       4.0f
#define PWM_LIMIT           100.0f

/* A wheel target below this is treated as "stopped" [rad/s]. */
#define W_ZERO_EPS          0.01f

/* ------------------------------------------------------------------------- */
/* UART protocol (frame layout as in the original; SCALE CHANGED to 0.02)     */
/* ------------------------------------------------------------------------- */
/*
 * Host -> MCU frame, 9600 8N1, 5 bytes:
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
#define UART_BAUD           9600
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
 * With the ROS convention (x forward, y left) the physical layout is ASSUMED to be
 * (NOT yet verified on the robot):
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
 * Motor input polarity: 1 inverts the direction pin for that motor, so that a
 * POSITIVE power drives the wheel FORWARD (the same sense as a positive
 * encoder speed). v1: M2 and M3 inverted, M1 and M4 unchanged (v0 had none inverted).
 * If a wheel runs the wrong way, flip its flag here.
 */
#define MOTOR1_DIR_INVERT   0
#define MOTOR2_DIR_INVERT   1
#define MOTOR3_DIR_INVERT   1
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
