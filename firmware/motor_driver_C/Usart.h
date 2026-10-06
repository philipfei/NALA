/*
 * Usart.h
 *
 * Created: 5/30/2020 4:40:26 PM
 *  Author: sojim
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06):
 *   - transmit path is now interrupt driven (ring buffer), see USART.c
 *   - added usart_try_send_buf() (non-blocking, drops if no room)
 *   - removed prototypes that were never implemented
 */
//Changed from Cpp to C by naming all funtions differently 3/21/2021

#ifndef USART_H_
#define USART_H_

#include <stdint.h>
#include <avr/io.h>
#define MAX_BUFFER 32

/* This structure handles the IO between the motor driver and the host
 * The data input is the received string, the data output is the buffer to
 * the data. Both are asynchronous, lastly when a newline is encountered the new command flag is set
 * (Kept from the original; not used by the NALA_v0 firmware.)
 */
typedef struct{
	volatile char data_input[MAX_BUFFER];
	volatile char data_ouput[MAX_BUFFER];
	volatile int new_command;
	volatile uint8_t byte_count;
}uart_io_t;

void usart_enable(uint16_t baudrate);
char usart_recieve(void);          /* blocking receive; NOT used by the control firmware */

/* Queue one byte for transmission. Returns immediately unless the TX buffer is full. */
void usart_send(char);
void usart_send_str(char *);
void usart_send_int(int);
void usart_send_int32(int32_t);
void usart_send_uint16(uint16_t);
void usart_send_uint32(uint32_t);
void usart_newline(void);

/*
 * Non-blocking: queue len bytes only if ALL of them fit, otherwise queue nothing.
 * Returns 1 if queued, 0 if dropped. Used for periodic telemetry so a slow
 * serial line can never stall the control loop.
 */
uint8_t usart_try_send_buf(const char *buf, uint8_t len);

#endif /* USART_H_ */
