/*
 * notused.c
 *
 * Created: 10/31/2021 4:58:03 PM
 *  Author: flori
 */ 
// 
// 
// void write_bit(uint8_t *PORT, uint8_t number, uint8_t bit) {
// 	if (bit>0) {
// 		*PORT |=  (1 << number);
// 		} else {
// 		*PORT &= ~(1 << number);
// 	}
// 	
// }
// 
// void write_state(uint8_t state) {
// 	if(counter == 328){counter = 1;}
// 	else{counter++;}
// 	
// 	if(counter == current_pos + 1 || ((counter == 1) && (current_pos = 328))       ){
// 		
// 		if(current_pos == 328){current_pos = 1;}
// 		else{current_pos++;}
// 		
// 		
// 		write_bit(&PORTD, 6, state & (1 << 0)); // EN1
// 		write_bit(&PORTD, 5, state & (1 << 1)); // EN2
// 		write_bit(&PORTE, 0, state & (1 << 2)); // PH1
// 		write_bit(&PORTE, 1, state & (1 << 3)); // PH2
// 		
// 		char str[200] = {0}; //edit flags in atmel studio to let this work
// 		//sprintf(str, "hallo \n");
// 		sprintf (str, "%d, %d, %d, %d, %d \n", counter, state & (1 << 0), state & (1 << 1), state & (1 << 2), state & (1 << 3));
// 		usart_send_str(str);
// 		
// 		if(current_pos == wanted_pos)
// 		{
// 			stop = 1;
// 			usart_send_str("STOP \n");
// 			//longjmp(env, 1);
// 		}
// 	}
// 	
// 	
// 	//char c[20] = {};
// 	//sprintf(c, "EN1 %d,EN2 %d, PH1 %d, PH2 %d \n", state & (1 << 0), state & (1 << 1), state & (1 << 2), state & (1 << 3));
// 	//usart_send_str(c);
// 	
// 	
// }
// 
// void loop_through()
// {
// 	while(1) {
// 		counter = 0;
// 		for(uint8_t i = 0; i < 4 && stop == 0; i++) {
// 			write_state(0);
// 			_delay_ms(500);
// 			for(uint8_t j = 0; j < 8 && stop == 0; j++) {
// 				
// 				
// 				uint8_t j_left  = j &  ((~0) << i);
// 				uint8_t j_right = j & ~((~0) << i);
// 				
// 				uint8_t other_state = (j_left << (i+1)) | j_right;
// 				
// 				if (stop == 0){write_state(other_state);}
// 				_delay_ms(delay_t);
// 				if (stop == 0){write_state(other_state | (1<<i));}
// 				_delay_ms(delay_t);
// 				if (stop == 0){write_state(other_state);}
// 				_delay_ms(delay_t);
// 				if (stop == 0){write_state(other_state | (1<<i));}
// 				_delay_ms(delay_t);
// 				if (stop == 0){write_state(other_state);}
// 				_delay_ms(delay_t);
// 			}
// 		}
// 	}
// }
// 
// //void m_printf(char* str)
// 
// void switch_direc()
// {
// 	char str[200] = {0};
// 	sprintf(str, "STARTING LOOP\n");
// 	usart_send_str(str);
// 	PORTD |= (1 << 5);
// 	PORTD &= ~(1 << 6);
// 	PORTE &= ~(1 << 0);
// 	PORTE &= ~(1 << 1);
// 
// 	// 		sprintf (str, "1\n");
// 	// 		usart_send_str(str);
// 	// 		_delay_ms(delay_t);
// 	// 		PORTE |= (1 << 0);
// 	// 		PORTE &= ~(1 << 1);
// 	// 		PORTD &= ~(1 << 5);
// 	// 		PORTD &= ~(1 << 6);
// 	// 		sprintf (str, "2\n");
// 	// 		usart_send_str(str);
// 	// 		_delay_ms(delay_t);
// 	// 		PORTE &= ~(1 << 0);
// 	// 		PORTE &= ~(1 << 1);
// 	// 		PORTD &= ~(1 << 5);
// 	// 		PORTD &= ~(1 << 6);
// 	// 		sprintf (str, "3\n");
// 	// 		usart_send_str(str);
// 	// 		_delay_ms(delay_t);
// 	PORTD &= ~(1 << 5);
// 	PORTD |= (1 << 6);
// 	PORTE &= ~(1 << 0);
// 	PORTE &= ~(1 << 1);
// 
// 	// 		sprintf (str, "4\n");
// 	// 		usart_send_str(str);
// 	// 		_delay_ms(delay_t);
// 	// 		PORTE &= ~(1 << 0);
// 	// 		PORTE &= ~(1 << 1);//
// 	// 		PORTD &= ~(1 << 5);//
// 	// 		PORTD &= ~(1 << 6);
// 	// 		sprintf (str, "5\n");
// 	// 		usart_send_str(str);
// 	// 		_delay_ms(delay_t);
// 	// 		PORTE &= ~(1 << 0);
// 	// 		PORTE &= ~(1 << 1);
// 	// 		PORTD &= ~(1 << 5);
// 	// 		PORTD &= ~(1 << 6);
// 	// 		_delay_ms(delay_t);
// 	// 		PORTE &= ~(1 << 0);
// 	// 		PORTE &= ~(1 << 1);
// 	// 		PORTD |= (1 << 5);
// 	// 		PORTD &= ~(1 << 6);
// 	// 		_delay_ms(delay_t);
// }
// 
// void goto_state(int wanted)
// {
// 	wanted_pos = wanted;
// 	loop_through();
// 	setjmp(env);
// 	usart_send_str("STOP2 \n");
// }
