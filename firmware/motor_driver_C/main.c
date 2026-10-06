/*
 * motor_driver_C.c
 *
 * Created: 9/18/2021 1:02:46 PM
 * Author : Floris van mourik
 */ 

#define F_CPU	16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <setjmp.h>

#define delay_t 10


//CLASSIC PID ZIEGLER NICHOLS:
// #define K_p 2.22 //Ku = 3.7
// #define K_i 11.1
// #define K_d 0.02775

//no overshoot PID ZIEGLER NICHOLS:
#define K_p 0.74 //Ku = 3.7
#define K_i 3.7
#define K_d 0.0644




#define R 0.04f
#define W 0.165f
#define H 0.15f

char str[200] = {0};

double M1_w;
double M2_w;
double M3_w;
double M4_w;

double M1_w_measured;
double M2_w_measured;
double M3_w_measured;
double M4_w_measured;

double old_int_error_1 = 0;
double old_error_1 = 0;
double old_int_error_2 = 0;
double old_error_2 = 0;
double old_int_error_3 = 0;
double old_error_3 = 0;
double old_int_error_4 = 0;
double old_error_4 = 0;

int M1_pwm = 0;
int M2_pwm = 0;
int M3_pwm = 0;
int M4_pwm = 0;


//Additional buffer size to accomodate accidental overflows?
#define BUF_SIZE 5

int8_t inputValues[BUF_SIZE] = {0}; 
volatile int counter = 0;
volatile uint8_t prevUARTval = 0;
volatile bool readVelCmd = false;

ISR(USART0_RX_vect)
{
	uint8_t currUARTval;
	currUARTval=UDR0;

	// usart_send_str("INSIDE INTERRUPT");

	if(readVelCmd) {
		inputValues[counter++] = currUARTval;	
		
		if(counter >= 3) {
			counter = 0;
			readVelCmd = false;
			prevUARTval = 0; //reset the flag byte checks variables.
			memset(inputValues, 0, BUF_SIZE);
			calc_angular_speed(((float) inputValues[0])/100.0f,
								((float) inputValues[1])/100.0f,
								((float) inputValues[2])/100.0f
							);
		}

		return;
	} 

	//Start flag = 0x8086
	if(prevUARTval == 0x80 && currUARTval == 0x86) {
		readVelCmd = true;
		counter = 0;
	}

	prevUARTval = currUARTval;
	
}


void calc_angular_speed(double Vx,double Vy, double w) //input speed in m/s and angular speed in rad/s
{
	sprintf (str, "%f %f %f\n", Vx, Vy, w);
	usart_send_str(str);
	M1_w = 1.0f/R*(1.0f*Vx - Vy - (W + H)*w);
	M2_w = 1.0f/R*(Vx + Vy - (W + H)*w);
	M3_w = 1.0f/R*(1.0f*Vx - Vy + (W + H)*w);
	M4_w = 1.0f/R*(Vx + Vy + (W + H)*w);
	
	M1_pwm = M1_w*4.0f;
	M2_pwm = M2_w*4.0f;
	M3_pwm = M3_w*4.0f;
	M4_pwm = M4_w*4.0f;
}


void set_motor_speed()
{

	
	if(M1_pwm > 100) {M1_pwm = 100;}
	if(M1_pwm < -100) {M1_pwm = -100;}
	if(M2_pwm > 100) {M2_pwm = 100;}
	if(M2_pwm < -100) {M2_pwm = -100;}
	if(M3_pwm > 100) {M3_pwm = 100;}
	if(M3_pwm < -100) {M3_pwm = -100;}
	if(M4_pwm > 100) {M4_pwm = 100;}
	if(M4_pwm < -100) {M4_pwm = -100;}
	
	set_motor_power(M1_pwm, M2_pwm, M3_pwm, M4_pwm);
}

double omega_measure()
{

	  		//usart_send_str("in looppp \n");
			  int aState_1;
			  int aLastState_1;
			  int aState_2;
			  int aLastState_2;
	  		int aState_3;
	  		int aLastState_3;
			int aState_4;
			int aLastState_4;
	
	  		// Reads the initial state of the outputA
			  aLastState_1 = (PINB & (1 << 3)) >> 3;
			  aLastState_2 = (PIND & (1 << 2)) >> 2;
	  		aLastState_3 = (PIND & (1 << 4)) >> 4;
			aLastState_4 = (PINC & (1 << 2)) >> 2;
			
			double M1_Encoder = 0;
			double M2_Encoder = 0;
			double M3_Encoder = 0;
			double M4_Encoder = 0;
			
			int a = 0;
	  		while(a < 1000) { //Add here the time!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  			//_delay_ms(0.1);
				
	  			aState_3 = (PIND & (1 << 4)) >> 4; // Reads the "current" state of the outputA
				aState_4 = (PINC & (1 << 2)) >> 2; // Reads the "current" state of the outputA
				
// 				while(1)
// 				{
// 					aState_4 = (PINC & (1 << 2)) >> 2;
// 					sprintf (str, "%d \n", aState_4);
// 					usart_send_str(str);
// 				}
				
	  			// If the previous and the current state of the outputA are different, that means a Pulse has occured
				  
				  
	  			if (aState_3 != aLastState_3){
	  				// If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
	  				if (((PIND & (1 << 7)) >> 7 )!= aState_3)
	  				{
	  					M3_Encoder = M3_Encoder + 1;
	  				}
	  				else
	  				{
	  					M3_Encoder = M3_Encoder - 1;
	  				}
	  				aLastState_3 = aState_3;
	  			}
				  
				  if (aState_4 != aLastState_4){
					  // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
					  if (((PINC & (1 << 3)) >> 3 )!= aState_4)
					  {
						  M4_Encoder = M4_Encoder + 1;
					  }
					  else
					  {
						  M4_Encoder = M4_Encoder - 1;
					  }
					  aLastState_4 = aState_4;
				  }
				 a++;
				 _delay_ms(0.01);
	 		}
			a = 0;
			while(a < 1000)
			{
				aState_2 = (PIND & (1 << 2)) >> 2; // Reads the "current" state of the outputA
				aState_1 = (PINB & (1 << 3)) >> 3; // Reads the "current" state of the outputA
				
				
				if (aState_1 != aLastState_1){
					// If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
					if (((PINB & (1 << 4)) >> 4 )!= aState_1)
					{
						M1_Encoder = M1_Encoder - 1;
					}
					else
					{
						M1_Encoder = M1_Encoder + 1;
					}
					aLastState_1 = aState_1;
				}
				
								  if (aState_2 != aLastState_2){
									  // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
									  if (((PIND & (1 << 3)) >> 3 )!= aState_2)
									  {
										  M2_Encoder = M2_Encoder - 1;
									  }
									  else
									  {
										  M2_Encoder = M2_Encoder + 1;
									  }
									  aLastState_2 = aState_2;
								  }
				a++;
				_delay_ms(0.01);
			}
			 
			 
	  		//sprintf (str, "Position: %f rounds\n", M1_Encoder/768*0.5);
	  		//usart_send_str(str);
			M1_w_measured = ((M1_Encoder/1536)/0.010*6.28); //in radians per second
			M2_w_measured = ((M2_Encoder/1536)/0.010*6.28); //in radians per second
			M3_w_measured = ((M3_Encoder/1536)/0.010*6.28); //in radians per second
			M4_w_measured = ((M4_Encoder/1536)/0.010*6.28); //in radians per second
			//sprintf (str, "speed: %f round/s\n", omega);
			//usart_send_str(str);
			
}


int PID_calc(double w_measured, double time, int motor)
{
	double err;
	double u;
	
	if(motor == 1)
	{
		err = w_measured - M1_w;
		
		double integral = err*time + old_int_error_1;
		double derivative = (err - old_error_1) / time;
		
		
		u =  K_p*err + K_i*integral + K_d*derivative;
		
		old_error_1 = err;
		old_int_error_1 = integral; //setting the old value to enable integrating the total error
	}
	
	if(motor == 2)
	{
		err = w_measured - M2_w;
		
		double integral = err*time + old_int_error_2;
		double derivative = (err - old_error_2) / time;
		
		
		u =  K_p*err + K_i*integral + K_d*derivative;
		
		old_error_2 = err;
		old_int_error_2 = integral; //setting the old value to enable integrating the total error
	}
	
	if(motor == 3)
	{
		err = w_measured - M3_w;
		
		double integral = err*time + old_int_error_3;
		double derivative = (err - old_error_3) / time;
		
		
		u =  K_p*err + K_i*integral + K_d*derivative;
		
		old_error_3 = err;
		old_int_error_3 = integral; //setting the old value to enable integrating the total error
	}
	if(motor == 4)
	{
		err = w_measured - M4_w;
		
		double integral = err*time + old_int_error_4;
		double derivative = (err - old_error_4) / time;
		
		u =  K_p*err + K_i*integral + K_d*derivative;
		
		old_error_4 = err;
		old_int_error_4 = integral; //setting the old value to enable integrating the total error
	}
	return u;
}


int main(void)
{
	sei();
	// cli();
	adc_init(0); //init with 5V ref
    usart_enable(9600);
	usart_send_str("test\n");
	

	
	
	
	DDRD |= (1 << 6); //pin D6 output
	PORTD &= ~(1 << 6);
	DDRD |= (1 << 5); //pin D5 output
	PORTD &= ~(1 << 5);
	DDRB |= (1 << 1); //pin D1 output
	PORTB &= ~(1 << 1);
	DDRB |= (1 << 2); //pin D2 output
	PORTB &= ~(1 << 2);
	
	DDRE |= (1 << 0); //pin E0 on output
	PORTE &= ~(1 << 0);
	DDRE |= (1 << 1); //pin E1 on output
	PORTE &= ~(1 << 1);
	DDRE |= (1 << 2); //pin E2 on output
	PORTE &= ~(1 << 2);
	DDRE |= (1 << 3); //pin E3 on output
	PORTE &= ~(1 << 3);
	
	DDRB &= ~(1 << 3); //pin 1,2,3,4 : a,b
	DDRB &= ~(1 << 4);
	DDRD &= ~(1 << 2);
	DDRD &= ~(1 << 3);
	DDRD &= ~(1 << 4);
	DDRD &= ~(1 << 7);
	DDRC &= ~(1 << 2);
	DDRC &= ~(1 << 3);
	
	
	
	init_motor_pins();
	init_motor_timers();
	set_motor_power(0,0,0,0);
	
	
// 	while(1)
// 	{
// 		start_timer3();
// 		uint16_t st = read_timer3();
// 		_delay_ms(1);
// 		
// 		uint16_t a = read_timer3();
// 		stop_timer3();
// 		sprintf (str, "timer: start: %u  end: %u \n",st, a);
// 		usart_send_str(str);
// 	}



// 
// int counter = 0;
// char ibp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// while(1) { //old function without PID for christmas holiday debugging
// 	usart_send_str("in loop \n");
// 	while(1) {
// 		int8_t inp = usart_recieve();
// 		
// 		// 			sprintf (str, "REC: %i\n", inp);
// 		// 			usart_send_str(str);
// 		if(inp == 1) {
// 			//usart_recieve();
// 			break;
// 		}
// 	}
// 	ibp[0] = usart_recieve();
// 	ibp[1] = usart_recieve();
// 	ibp[2] = usart_recieve();
// 	//ibp[3] = usart_recieve();
// 	
// 	int8_t Vx_cm = ibp[0];
// 	int8_t Vy_cm = ibp[1];
// 	int8_t W_mo = ibp[2];
// 	
// 	double Vy = Vy_cm/100.0f;
// 	double Vx = Vx_cm/100.0f;
// 	double w = W_mo/10.0f;
// 	
// 	
// 	
// 	calc_angular_speed(Vx, Vy, w);
// 	set_motor_speed();
// 	// 		sprintf (str, "%f %f %f | %f %f %f %f\n", Vx, Vy, w, M1_w, M2_w, M3_w, M4_w);
// 	// 		usart_send_str(str);
// 	
// 	//usart_recieve();
// 	//usart_send_str("in loop 2 \n");
// 	//usart_send('R');
// 	
// 	//usart_recieve();
// }




	
	
	/*
 	while(1) //while loop for PID testing, do not remove or change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 	{
 		double Vx = 0.0;
 		double Vy = 0.1;
		double w = 0;
 		
 		
 		calc_angular_speed(Vx, Vy, w);
 		set_motor_speed();
 		
 		int m1_old = M1_pwm;
 		int m2_old = M2_pwm;
 		int m3_old = M3_pwm;
 		int m4_old = M4_pwm;
 		
 		// 		sprintf (str, "%f\n", M2_w);
 		// 		usart_send_str(str);
 		// 		_delay_ms(999999);
 		int check = 0;
 		int hu = 0;
 		while(1)
 		{
 			hu++;
 			
 			omega_measure();
 			double time_measured = 0.025;
 			
 			int u1 = PID_calc(M1_w_measured, time_measured, 1);
 			int u2 = PID_calc(M2_w_measured, time_measured, 2);
 			int u3 = PID_calc(M3_w_measured, time_measured, 3);
 			int u4 = PID_calc(M4_w_measured, time_measured, 4);
 			
 			M1_pwm = M1_pwm - u1;
 			M2_pwm = M2_pwm - u2;
 			M3_pwm = M3_pwm - u3;
 			M4_pwm = M4_pwm - u4;
 			
 			if(hu > 100)
 			{
 				hu = 0;
 				check = 1;
 			}
 			
 			
 			if(check == 1 && (M1_pwm > 40 || M1_pwm < -40 || M2_pwm > 40 || M2_pwm < -40 || M3_pwm > 40 || M3_pwm < -40 || M4_pwm > 40 || M4_pwm < -40))
 			{
 				M1_w = -1*M1_w;
 				M2_w = -1*M2_w;
 				M3_w = -1*M3_w;
 				M4_w = -1*M4_w;
 				check = 0;
 			}
 			
 			
 			
 			//sprintf (str, "%d \t %d \t %f \t %f\n", M1_pwm, u1, M1_w_measured, M1_w);
 			//sprintf (str, "%d \t %d \t %f \t %f\n", M2_pwm, u2, M2_w_measured, M2_w);
 			//sprintf (str, "%d \t %d \t %f \t %f\n", M3_pwm, u3, M3_w_measured, M3_w);
 			sprintf (str, "%d \t %d \t %f \t %f\n", M4_pwm, u4, M4_w_measured, M4_w);
 			//sprintf (str, "%f \t %f \t %f \t %f\n", M1_w_measured, M2_w_measured, M3_w_measured, M4_w_measured);
 			usart_send_str(str);
 			set_motor_speed();
 			_delay_ms(5);
 		}
 	}
 	*/

	
	
	

		
	
	calc_angular_speed(0, 0, 0);
	set_motor_speed();
	while(1)
	{
		omega_measure();
		double time_measured = 0.045;
		
		int u1 = PID_calc(M1_w_measured, time_measured, 1);
		int u2 = PID_calc(M2_w_measured, time_measured, 2);
		int u3 = PID_calc(M3_w_measured, time_measured, 3);
		int u4 = PID_calc(M4_w_measured, time_measured, 4);
		
		M1_pwm = M1_pwm - u1;
		M2_pwm = M2_pwm - u2;
		M3_pwm = M3_pwm - u3;
		M4_pwm = M4_pwm - u4;

		
		//sprintf (str, "%d \t %d \t %f \t %f\n", M1_pwm, u1, M1_w_measured, M1_w);
		//sprintf (str, "%d \t %d \t %f \t %f\n", M2_pwm, u2, M2_w_measured, M2_w);
		//sprintf (str, "%d \t %d \t %f \t %f\n", M3_pwm, u3, M3_w_measured, M3_w);
		//sprintf (str, "%d \t %d \t %f \t %f\n", M4_pwm, u4, M4_w_measured, M4_w);
		sprintf (str, "%f \t %f \t %f \t %f\n", M1_w_measured, M2_w_measured, M3_w_measured, M4_w_measured);
		usart_send_str(str);
		set_motor_speed();
		_delay_ms(20);
	}
}

