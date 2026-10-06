/*
 * encoder.c  --  interrupt-driven quadrature encoder counting (NALA_v0)
 *
 * Author  : HY (NALA v0 revision)
 * Version : v0.1.0
 * Date    : 2026-10-06
 *
 * Replaces the blocking polling windows of the original omega_measure().
 * Counting rule is identical to the original:
 *   - an edge on channel A (rising or falling) is one count,
 *   - direction: +1 if channel B != channel A (new level), else -1.
 * so ENC_COUNTS_PER_REV (config.h) stays valid.
 *
 * Every pin-change interrupt re-scans all four encoders and compares channel A
 * with its previous level. This keeps the ISR short and independent of which
 * port the interrupt came from.
 */

#include "config.h"
#include "encoder.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

static volatile int32_t enc_raw[ENC_COUNT];
static volatile uint8_t enc_last_a[ENC_COUNT];

/* Read channel A, and if it changed since last time, count one step. */
#define ENC_STEP(n)                                                         \
    do {                                                                    \
        uint8_t a_ = (ENC##n##_A_PINREG >> ENC##n##_A_BIT) & 1u;            \
        if (a_ != enc_last_a[(n) - 1]) {                                    \
            uint8_t b_ = (ENC##n##_B_PINREG >> ENC##n##_B_BIT) & 1u;        \
            enc_raw[(n) - 1] += (b_ != a_) ? 1 : -1;                        \
            enc_last_a[(n) - 1] = a_;                                       \
        }                                                                   \
    } while (0)

static inline void encoder_scan(void)
{
    ENC_STEP(1);
    ENC_STEP(2);
    ENC_STEP(3);
    ENC_STEP(4);
}

/* Pin-change vectors: PCINT0 = port B, PCINT1 = port C, PCINT2 = port D. */
ISR(PCINT0_vect) { encoder_scan(); }
ISR(PCINT1_vect) { encoder_scan(); }
ISR(PCINT2_vect) { encoder_scan(); }

void encoder_init(void)
{
    /* Inputs, pull-ups off (A and B of every encoder). */
    ENC1_DDR &= ~(_BV(ENC1_A_BIT) | _BV(ENC1_B_BIT));
    ENC2_DDR &= ~(_BV(ENC2_A_BIT) | _BV(ENC2_B_BIT));
    ENC3_DDR &= ~(_BV(ENC3_A_BIT) | _BV(ENC3_B_BIT));
    ENC4_DDR &= ~(_BV(ENC4_A_BIT) | _BV(ENC4_B_BIT));

    /* Remember the current A levels so the first interrupt does not count a phantom edge. */
    enc_last_a[0] = (ENC1_A_PINREG >> ENC1_A_BIT) & 1u;
    enc_last_a[1] = (ENC2_A_PINREG >> ENC2_A_BIT) & 1u;
    enc_last_a[2] = (ENC3_A_PINREG >> ENC3_A_BIT) & 1u;
    enc_last_a[3] = (ENC4_A_PINREG >> ENC4_A_BIT) & 1u;

    /* Pin-change interrupt on the A pins only. */
    ENC1_A_PCMSK |= _BV(ENC1_A_PCINT);
    ENC2_A_PCMSK |= _BV(ENC2_A_PCINT);
    ENC3_A_PCMSK |= _BV(ENC3_A_PCINT);
    ENC4_A_PCMSK |= _BV(ENC4_A_PCINT);

    PCIFR = _BV(PCIF0) | _BV(PCIF1) | _BV(PCIF2);   /* clear stale flags */
    PCICR |= _BV(ENC1_PCIE) | _BV(ENC2_PCIE) | _BV(ENC3_PCIE) | _BV(ENC4_PCIE);
}

void encoder_read_raw(int32_t out[ENC_COUNT])
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < ENC_COUNT; i++) {
            out[i] = enc_raw[i];
        }
    }
}
