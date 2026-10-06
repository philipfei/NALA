#define F_CPU	16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define K_p 100
#define K_i 100
#define K_d 100

/**
 * @brief Set the the motor powers. Input should be given between 0-100.
 * 
 * @param m1 Power for motor 1.
 * @param m2 Power for motor 2.
 * @param m3 Power for motor 3.
 * @param m4 Power for motor 4.
 
 
 */




void set_motor_power(int m1, int m2, int m3, int m4) {
    /*
    //Ensuring the input is in the correct range.
    //Could be a bit slow because of multiple checks. Not sure if it matters much here.
    if(m1 <= 0) m1 = 0;
    if(m2 <= 0) m2 = 0;
    if(m3 <= 0) m3 = 0;
    if(m4 <= 0) m4 = 0;

    if(m1 >= 100) m1 = 100;
    if(m2 >= 100) m2 = 100;
    if(m3 >= 100) m3 = 100;
    if(m4 >= 100) m4 = 100;
    */
	
	
	m1 = -1*m1; //for reversing the motor with position
	m2 = -1*m2;
	m4 = -1*m4;
	m3 = -1*m3;
	if(m1 < 0)
	{
		m1 = -1*m1;
		PORTE &= ~(1 << 0);
	}
	else
	{
		PORTE |= (1 << 0);
	}
	
	if(m2 < 0)
	{
		m2 = -1*m2;
		PORTE &= ~(1 << 1);
	}
	else
	{
		PORTE |= (1 << 1);
	}
	
	if(m3 < 0)
	{
		m3 = -1*m3;
		PORTE &= ~(1 << 2);
	}
	else
	{
		PORTE |= (1 << 2);
	}
	
	if(m4 < 0)
	{
		m4 = -1*m4;
		PORTE &= ~(1 << 3);
	}
	else
	{
		PORTE |= (1 << 3);
	}
	
    //Be careful of integer / integer division.
    OCR0A = (int) ((100-m1)/100.0f * 0xFF); //PD6 (m1)
    OCR0B = (int) ((100-m2)/100.0f * 0xFF); //PD5 (m2)

    //Using the low byte register because we are using 8-bit PWM for the 16-bit timers
    OCR1AL = (int) ((100-m3)/100.0f * 0xFF); //PB1 (m3)
    OCR1BL = (int) ((100-m4)/100.0f * 0xFF); //PB2 (m4)
}
