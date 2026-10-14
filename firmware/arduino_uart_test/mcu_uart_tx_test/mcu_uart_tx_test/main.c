/*
 * mcu_uart_tx_test: bench test of the MCU UART TX line. Not part of the robot firmware.
 *
 * Sends a 21-byte frame again and again on UART0 (PD1 = TXD0), 38400 8N1, 16 MHz,
 * one frame every 50 ms (period about 55.5 ms with the 5.5 ms frame):
 *   A5 02 seq flags  01 00 00 00  02 00 00 00  03 00 00 00  04 00 00 00  crc8
 *   sync id           M1=1         M2=2         M3=3         M4=4
 * It is a valid v2.1.0 state frame (crc8 poly 0x07 over bytes 1-19), so the Arduino sketch
 * (../arduino_uart_test.ino) can decode it.
 * Diagnostics (to tell an MCU reset from a clock problem):
 *   seq   = +1 per frame since the last reset. If it keeps starting again at 0, the MCU resets.
 *   flags = bits 0-3: reset cause read at start (MCUSR: 1 power-on, 2 external, 4 brown-out,
 *           8 watchdog); bit 7: XFDIF, set when the clock failure detection (CFD fuse) has
 *           switched the CPU from the crystal to the internal 1 MHz clock.
 * It does not read anything. The motors and encoders are not touched.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <util/delay.h>

#define UART_BAUD     38400UL
#define UBRR_VALUE    (F_CPU / (16UL * UART_BAUD) - 1UL)   /* 25 */
#define FRAME_MS      50
#define FRAME_LEN     21
#define SEQ_POS       2
#define FLAGS_POS     3
#define FLAG_XFDIF    0x80

static uint8_t frame[FRAME_LEN] = {
	0xA5, 0x02,                 /* sync, id */
	0x00, 0x00,                 /* seq, flags (filled per frame) */
	0x01, 0x00, 0x00, 0x00,     /* M1 */
	0x02, 0x00, 0x00, 0x00,     /* M2 */
	0x03, 0x00, 0x00, 0x00,     /* M3 */
	0x04, 0x00, 0x00, 0x00,     /* M4 */
	0x00                        /* crc8 (filled per frame) */
};

static void uart_init(void)
{
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
	UCSR0B = (1 << TXEN0);                       /* transmitter only */
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      /* 8 data bits, no parity, 1 stop bit */
}

static void uart_send(uint8_t b)
{
	while (!(UCSR0A & (1 << UDRE0))) {
	}
	UDR0 = b;
}

static uint8_t crc8_update(uint8_t crc, uint8_t b)
{
	crc ^= b;
	for (uint8_t i = 0; i < 8; i++) {
		crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
	}
	return crc;
}

int main(void)
{
	uint8_t reset_cause = MCUSR & 0x0F;
	MCUSR = 0;
	wdt_disable();          /* after a watchdog reset the watchdog stays on */

	uart_init();

	uint8_t seq = 0;
	for (;;) {
		frame[SEQ_POS] = seq++;
		frame[FLAGS_POS] = reset_cause | ((XFDCSR & (1 << XFDIF)) ? FLAG_XFDIF : 0);
		uint8_t crc = 0;
		for (uint8_t i = 1; i < FRAME_LEN - 1; i++) {
			crc = crc8_update(crc, frame[i]);
		}
		frame[FRAME_LEN - 1] = crc;

		for (uint8_t i = 0; i < FRAME_LEN; i++) {
			uart_send(frame[i]);
		}
		_delay_ms(FRAME_MS);
	}
}
