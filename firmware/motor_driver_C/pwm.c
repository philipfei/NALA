/*
 * pwm.c
 *
 * Created: 9/24/2021 2:16:34 PM
 *  Author: Mathan
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06): only the header include was added;
 * the PWM setup itself is unchanged.
 */
#include "config.h"
#include "pwm.h"

#include <avr/io.h>

//Motor pins
#define _1_EN PD6
#define _2_EN PD5
#define _3_EN PB1
#define _4_EN PB2

void init_motor_pins() {

    //Set pins to output mode.
    DDRD |= _BV(_1_EN) | _BV(_2_EN); //PD6 and PD5
    DDRB |= _BV(_3_EN) | _BV(_4_EN); //PB1 and PB2

    //PORTxn bits tells if you 1 is high or 0 is high
    //i.e. If PORTxn is written to '1' when the pin is configured as an output pin, the port pin is driven high.
}


/**
 * PB1 and PB2 uses timer1. PB1 uses A output and PB2 uses B output.
 * PD5 and PD6 uses timer0. PD5 uses B output and PD6 uses A output.
 */
void init_motor_timers() {
    //Timer 0 Output compare registers (8 bit value. Max: 255 decimal)
    OCR0A = 0;
    OCR0B = 0;

    //Set OC0A on Compare Match, clear OC0A at BOTTOM (inverting mode)
    //enables PWM alternate function for timer 0 ports [PD5 and PD6]
    TCCR0A |= _BV(COM0A0) | _BV(COM0A1);
    TCCR0A |= _BV(COM0B0) | _BV(COM0B1);

    //Set timer mode to Fast PWM
    TCCR0A |= _BV(WGM00) | _BV(WGM01);

    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaler
     TCCR0B |= _BV(CS00); 
     TCCR0B &= ~_BV(CS01) & ~_BV(CS02);

    
    //External clock source on T0 pin. Clock on rising edge. [To clock on failling edge, set CS00 to 0]
   // TCCR0B |= _BV(CS02) | _BV(CS01) | _BV(CS00);
    



    /**********TIMER 1****************/
    //Timer 1 Output compare registers (16 bit value. Max: 0xFFFF hex)
    OCR1A = 0;
    OCR1B = 0;

    //Set OC1A/OC1B on Compare Match, clear OC1A/OC1B at BOTTOM (inverting mode)
    //enables PWM alternate function for timer 1 ports [PD5 and PD6]
    TCCR1A |= _BV(COM1A0) | _BV(COM1A1); 
    TCCR1A |= _BV(COM1B0) | _BV(COM1B1); 

    //Set timer mode to Fast PWM, 8-bit
    //(WGM)13 12 11 10
    //      0  1  0  1 
    TCCR1A |= _BV(WGM10);
    TCCR1A &= ~_BV(WGM11);
    TCCR1B |= _BV(WGM12);
    TCCR1B &= ~_BV(WGM13);

    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaler
     TCCR1B |= _BV(CS10); 
     TCCR1B &= ~_BV(CS11) & ~_BV(CS12);

    
    //External clock source on T1 pin. Clock on rising edge. [To clock on failling edge, set CS10 to 0]
    //TCCR1B |= _BV(CS12) | _BV(CS11) | _BV(CS10);
    
}
