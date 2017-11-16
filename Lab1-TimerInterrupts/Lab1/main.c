/*########################################################################
# MILESTONE : 1
# PROGRAM : 1
# PROJECT : MyFirstProject
# GROUP : X
# NAME 1 : First Name, Last Name, Student ID
# NAME 2 : First Name, Last Name, Student ID
# DESC : This program does…[write out a brief summary]
# DATA
# REVISED
########################################################################	*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <util/delay_basic.h>	// 
#include "Globals.h"

// LED Sequence Starting State
uint8_t LED_DIRECTION = RIGHT;	// Direction of the sequence
uint8_t led_counter = 0;		// Track the sequence	 


void delaynus(int n) //%%% delay microsecond
{
	int k;
	for(k=0;k<n;k++)
	_delay_loop_1(1);
}

void delaynms(int n) // %%%delay millisecond
{
	int k;
	for(k=0;k<n;k++)
	delaynus(1000);
}

/* LedSequence
** 
** INPUTS
** port - a pointer to the port we want to supply power to
**
** DESCRIPTION
** This function shifts two high pins right to left and back.
** It is designed to display a a shifting sequence on LEDs.
** The sequence starts with one LED lit at the left and shifts 
** the lit led right (while adding a second lit LED) until one
** LED on the far right remains lit. The sequence then goes back
** to the left and so on.
*/
void ledSequence(volatile uint8_t *port) {	
	
	// LED sequence right
	if ((LED_DIRECTION == RIGHT)) {
		*port >> 1;
		
		// Add a second powered LED on the left
		if (led_counter == 0) {
			*port += 0x80;
			
		// Change direction at the end of the line
		} else if (led_counter == 7) {
			LED_DIRECTION = LEFT;
		}
		led_counter++;
		
	} else {
		*port << 1;
		
		// Add a second powered LED on the right
		if (led_counter == 8) {
			*port += 1;
			
		// Change direction at the end of the line
		} else if (led_counter == 1) {
			LED_DIRECTION = RIGHT;
		}
		led_counter--;
	}
	
}

//################## MAIN ROUTINE ##################
int main(int argc, char *argv[]){
	DDRC = 0b11111111; /* Sets all pins on Port C to output */
	PORTC = 0b10000000; /* initialize port to high – turn on LEDs */
	
	while(1) 
	{
		delaynms(25);
		ledSequence(&PORTC);	
	}
	
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

