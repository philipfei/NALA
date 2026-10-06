/*
 * Uart.cpp
 * Various UART functions
 * Created: 5/30/2020 4:46:40 PM
 *  Author: sojim
 *
 * Changed from Cpp to C by naming all funtions differently 3/21/2021
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06):
 *   The original usart_send() busy-waited on UDRE for every byte, so printing one
 *   telemetry line (~45 bytes at 9600 baud, ~1 ms per byte) blocked the CPU for
 *   ~45 ms. TX now goes through a ring buffer drained by the UDRE interrupt:
 *   usart_send() only waits if the buffer is completely full.
 *   RX is unchanged (the receive ISR lives in main.c).
 */

#include "config.h"
#include "Usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define USART_TX_BUF_SIZE 128u              /* must be a power of two, <= 256 */
#define USART_TX_MASK     (USART_TX_BUF_SIZE - 1u)

static volatile uint8_t tx_buf[USART_TX_BUF_SIZE];
static volatile uint8_t tx_head;            /* next free slot (written by main code) */
static volatile uint8_t tx_tail;            /* next byte to send (written by the ISR) */

/* Move one byte from the ring buffer to the UART if the data register is free. */
static void tx_pump(void)
{
	if (tx_head != tx_tail && (UCSR0A & (1 << UDRE0))) {
		UDR0 = tx_buf[tx_tail];
		tx_tail = (tx_tail + 1u) & USART_TX_MASK;
	}
}

ISR(USART0_UDRE_vect)
{
	if (tx_head == tx_tail) {
		UCSR0B &= ~(1 << UDRIE0);           /* nothing left: stop UDRE interrupts */
	} else {
		UDR0 = tx_buf[tx_tail];
		tx_tail = (tx_tail + 1u) & USART_TX_MASK;
	}
}

void usart_enable(uint16_t baudrate){
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable the USART Transmitter and  receive interrupt
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 data bits, 1 stop bit */

	baudrate = F_CPU/(16.0*baudrate) -1;
	UBRR0H=baudrate >> 8;
	UBRR0L=baudrate & 0xFF;

	tx_head = 0;
	tx_tail = 0;
}
//Blocking USART receive (kept from the original, unused by the control firmware)
char usart_recieve(void){
	UCSR0B &= ~(1 << RXCIE0 ); //turn off interrupts
	while(~UCSR0A & (1<<RXC0));
	char c = UDR0;
	UCSR0B |= (1 << RXCIE0 ); //turn on interrupts
	return  c;

}

void usart_newline(void){
	usart_send('\n');
}

//Send character (queued; blocks only while the TX buffer is full)
void usart_send(char character){
	uint8_t next = (tx_head + 1u) & USART_TX_MASK;
	while (next == tx_tail) {
		/* Buffer full. If interrupts are globally off (e.g. during start-up) the ISR
		 * can not drain it, so drain it by hand to avoid a dead-lock. */
		if (!(SREG & (1 << SREG_I))) {
			tx_pump();
		}
	}
	tx_buf[tx_head] = (uint8_t)character;
	tx_head = next;
	UCSR0B |= (1 << UDRIE0);
}

uint8_t usart_try_send_buf(const char *buf, uint8_t len)
{
	uint8_t free_slots = (uint8_t)((tx_tail - tx_head - 1u) & USART_TX_MASK);
	if (len > free_slots) {
		return 0;
	}
	for (uint8_t i = 0; i < len; i++) {
		tx_buf[tx_head] = (uint8_t)buf[i];
		tx_head = (tx_head + 1u) & USART_TX_MASK;
	}
	UCSR0B |= (1 << UDRIE0);
	return 1;
}

//Send integer
void usart_send_int(int integer){
	char buffer[10];
	itoa(integer,buffer,10);
	for(int i = 0; buffer[i] != 0; i++){
		usart_send(buffer[i]);
	}
}
void usart_send_int32(int32_t integer){
	char buffer[11];
	ltoa(integer,buffer,10);
	for(int i = 0; buffer[i] != 0; i++){
		usart_send(buffer[i]);
	}
}
void usart_send_uint16(uint16_t number){
	char buffer[10];
	utoa(number,buffer,10);
	for(int i = 0; buffer[i] !=0; i++){
		usart_send(buffer[i]);
	}
}
void usart_send_uint32(uint32_t number){
	char buffer[11];
	ultoa(number,buffer,10);
	for(int i = 0; buffer[i] !=0; i++){
		usart_send(buffer[i]);
	}
}
//Send string
void usart_send_str(char * text){
	char * index = text;
	for( ; *index != 0; index++){
		usart_send(*index);
	}

}
