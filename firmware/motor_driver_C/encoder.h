/*
 * encoder.h  --  interrupt-driven quadrature encoder counting (NALA_v0)
 *
 * Author  : HY (NALA v0 revision)
 * Version : v0.1.0
 * Date    : 2026-10-06
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

#define ENC_COUNT 4

/* Configure encoder pins as inputs (no pull-ups) and enable pin-change interrupts. */
void encoder_init(void);

/*
 * Copy the running RAW count of every encoder channel (index 0..3 = E1..E4).
 * Raw = +1 per A-edge when B != A, -1 otherwise (NO per-motor sign applied;
 * see MOTORn_ENC_SIGN in config.h). The counters never reset, differences
 * between two snapshots give the counts in between. Safe to call from any context.
 */
void encoder_read_raw(int32_t out[ENC_COUNT]);

#endif /* ENCODER_H_ */
