/*
 * Uart.h
 *
 * Created: 5/30/2020 4:40:26 PM
 *  Author: sojim
 */ 
//Changed from Cpp to C by naming all funtions differently 3/21/2021

#ifndef USART_H_
#define USART_H_

#include <avr/io.h> 
#define MAX_BUFFER 32 

/* This structure handles the IO between the motor driver and the host 
 * The data input is the received string, the data output is the buffer to 
 * the data. Both are asynchronous, lastly when a newline is encountered the new command flag is set
 */
typedef struct{
	volatile char data_input[MAX_BUFFER];
	volatile char data_ouput[MAX_BUFFER];
	volatile int new_command;	
	volatile uint8_t byte_count;
}uart_io_t;

void usart_enable(uint16_t baudrate) ;
char usart_fifo();
char usart_recieve();
void usart_unblock();
void usart_block();

void usart_send_int(int); 
void usart_send(char);
//void usart_send(float,int);
void usart_send_str(char *);
void usart_send_int32(int32_t);
void usart_send_uint16(uint16_t);
void usart_send_uint32(uint32_t);


void usart_newline();
#endif /* USART_H_ */