/*
 * timer.c
 *
 * Created: 12/21/2021 4:17:34 PM
 *  Author: Mathan
 */ 
#include <avr/io.h>


void start_timer3() {
    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaling
    TCCR3B |= _BV(CS31); 
    TCCR3B |= (_BV(CS30) | _BV(CS32));
}

//Might change return type if necessary
uint16_t read_timer3() {
    return ((TCNT3H << 8) | TCNT3L);
}

void stop_timer3() {
    TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
}

/*
 * 
 *  Doing the same as above for timer4 
 * 
 */
void start_timer4() {
    //Timer control register used to set clock source and prescaling.
    //Using inbuilt clock with no prescaling
    TCCR4B |= _BV(CS40); 
    TCCR4B &= ~(_BV(CS41) | _BV(CS42));
}

//Might change return type if necessary
uint16_t read_timer4() {
    return ((TCNT4H << 8) | TCNT4L);
}

void stop_timer4() {
    TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
}
