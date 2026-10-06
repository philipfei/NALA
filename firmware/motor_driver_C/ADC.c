/*
 * ADC.c
 * ADC function for reading the analog thingies
 * Created: 2/19/2021 2:26:13 PM
 * Author: FlorisvM
 *
 * NALA_v0 revision (HY, v0.1.0, 2026-10-06): only the includes were completed
 * (this file used usart_send()/usart_newline() without declaring them) and the
 * duplicate F_CPU definition was moved to config.h. Logic is unchanged.
 * The ADC is not used by the control loop; adc_init(0) is still called from
 * main() to keep the original start-up output ('a').
 */
#include "config.h"
#include "ADC.h"
#include "Usart.h"

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#define ADMUX_REF_AREF ((0 << REFS1)|(0 << REFS0))
#define ADMUX_REF_AVCC ((0 << REFS1)|(1 << REFS0))
#define ADMUX_REF_RESV ((1 << REFS1)|(0 << REFS0))
#define ADMUX_REF_VBG  ((1 << REFS1)|(1 << REFS0))

void adc_init(int mode){ //mode = 0 -> 5V ref, mode = 1 -> 1.1V ref

	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
	if (mode == 0){usart_send('a'); usart_newline(); ADMUX |= ADMUX_REF_AVCC;} //change the reference voltage to 5 or 1.1. Tricky because the 2.5V can not be used!!
	if (mode == 1){usart_send('b'); usart_newline(); ADMUX |= ADMUX_REF_VBG;}
	ADCSRA |= (1<<ADEN);                //Turn on ADC
	ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}

int read_adc(uint8_t channel){
	ADMUX &= 0xF0;                    //Clear the older channel that was read
	ADMUX |= channel;                //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
	return ADCW;                    //Returns the ADC value of the chosen channel
}
