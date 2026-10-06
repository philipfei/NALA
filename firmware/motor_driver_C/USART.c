/*
 * Uart.cpp
 * Various UART functions 
 * Created: 5/30/2020 4:46:40 PM
 *  Author: sojim
 */ 
//Changed from Cpp to C by naming all funtions differently 3/21/2021

#include "Usart.h"	
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#define F_CPU 16000000UL


void usart_enable(uint16_t baudrate){
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable the USART Transmitter and  receive interrupt
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 data bits, 1 stop bit */
	
	baudrate = F_CPU/(16.0*baudrate) -1;	
	UBRR0H=baudrate >> 8;
	UBRR0L=baudrate & 0xFF; 
}
//Blocking USART receive 
char usart_recieve(){
	UCSR0B &= ~(1 << RXCIE0 ); //turn off interrupts  
	while(~UCSR0A & (1<<RXC0));
	char c = UDR0; 
	UCSR0B |= (1 << RXCIE0 ); //turn on interrupts 
	return  c; 
		
}

void usart_newline(){
	usart_send('\n');	
}

//Send character 
void usart_send(char character){
	//wait till register is empty 
	while (~UCSR0A & (1 << UDRE0));
	UDR0 = character;
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

