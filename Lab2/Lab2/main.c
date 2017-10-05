/*
########################################################################
# MILESTONE : 2
# PROGRAM : 2
# PROJECT : Lab 2
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program outputs the "Knightrider" pattern to eight LEDs, using the MCU timer
# DATA
# REVISED
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <util/delay_basic.h> // header for delay library
#include <avr/interrupt.h> //header for interrupt function library

void mTimer(int count)
{
	int i=0;
	TCCR1B |= _BV(WGM12);	// Set WGM bits to 0100, see pg 142
	OCR1A = 0x03E8;			// Set output compare register for 1000 cycles  = 1ms
	TCNT1 = 0x0000;			// Set initial value of Timer Counter to 0x000
	TIMSK1 |= 0b00000010;   // Output compare interrupt enable --remove??
	TIFR1 |= _BV(OCF1A);	// Clear timer interrupt flag and begin timer
	
	/* Poll the timer to determine when the timer has reached 0x03E8 */
	while(i<count)
	{
		if((TIFR1 & 0x02) == 0x02)
		{
			// clear interrupt flag by writing a 1 to the bit
			TIFR1 |= _BV(OCF1A);
			i++;
		}
	}
	return; 
}/*mTimer*/

/*
INPUTS:
	d = output port
	
Description:
This function sets the output port pins in a "knightrider" pattern starting from left to right
The output port register is first set to one; lighting up the leftmost LED.
Then, the value is shifted and incremented or decremented so the pattern travels 
from L to R, disappears off the right side, and travels back towards the left.

*/
void pattern(volatile uint8_t* d) 
{
	*d = 0b00000001; // set output port to one, Px1 = 1
	int k;
		for(k=0;k<20;k++)
		{						
			mTimer(200);						// delay 200ms using MCU timer
			/*
			if(k<3)			*d = (*d<<1)+1;		// shift and add 1 to turn on an additional pin, so the number of lit LEDs increases to 4
			else if(k<8)	*d = (*d<<1);		// shift the values so the 4 LEDs travel from L > R
			else if(k<11)	*d = (*d-128)<<1;	// continue shifting, eliminating the rightmost bit to prevent overflow
			else if(k<14)	*d = (*d>>1)+128;   // shift back L, add 128 = Px7, so the rightmost LED lights again
			else if(k<18)	*d = (*d>>1);		// continue shifting, LEDs travel R > L
			else if(k<20)	*d = (*d-1)>>1;		// eliminate leftmost bit, shift back L until only Px1 remains lit
			*/
			if(k<3)			*d = (*d<<1)+1;		// shift and add 1 to turn on an additional pin, so the number of lit LEDs increases to 4
			else if(k<10)	*d = (*d<<1);		// shift the values so the 4 LEDs travel from L > R
			else if(k<13)	*d = (*d>>1)+128;   // shift back L, add 128 = Px7, so the rightmost LED lights again
			else if(k<20)	*d = (*d>>1);		// eliminate leftmost bit, shift back L until only Px1 remains lit
			
		}
}/*pattern*/

/*################## MAIN ROUTINE ##################*/
int main(int argc, char *argv[]){
	/* Timer Instructions */
	/* Set timer 1 to run at CPU clock, disable all function and use as pure timer */
	
	TCCR1B |=  _BV(CS10); // _BV sets the bit to logic 1, CS10 selects clock with no prescaling, f=1 MHz
	DDRC = 0xff;		  //Sets all pins on Port C to output */
	PORTC = 0x01;		  // Set bit 0 to high; light leftmost LED
	while (1)
	{
		pattern(&PORTC); // run the Knightrider pattern continuously
	}
	
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}



