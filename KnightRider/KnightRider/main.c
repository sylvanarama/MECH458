
/*
########################################################################
# MILESTONE : 1
# PROGRAM : 1
# PROJECT : Lab 1 part 4 - Knightrider
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program outputs the "knightrider" pattern to eight LEDs
# DATA
# REVISED
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <util/delay_basic.h> // header for delay library

void delaynus(int n) //delay microsecond
{
	int k;
	for(k=0;k<n;k++)
	_delay_loop_1(1);
}
void delaynms(int n) //delay millisecond
{
	int k;
	for(k=0;k<n;k++)
	delaynus(1000);
}

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
			if(k<4)			*d = (*d<<1)+1;		// shift and add 1 to turn on an additional pin, so the number of lit LEDs increases to 4
			else if(k<8)	*d = (*d<<1);		// shift the values so the 4 LEDs travel from L > R
			else if(k<11)	*d = (*d-128)<<1;	// continue shifting, eliminating the rightmost bit to prevent overflow
			else if(k<14)	*d = (*d>>1)+128;   // shift back L, add 128 = Px7, so the rightmost LED lights again
			else if(k<18)	*d = (*d>>1);		// continue shifting, LEDs travel R > L
			else if(k<20)	*d = (*d-1)>>1;		// eliminate leftmost bit, shift back L until only Px1 remains lit
			delaynms(15);						// delay before next loop so changes can be seen
		}//for
}//pattern

/*################## MAIN ROUTINE ##################*/
int main(int argc, char *argv[]){
	DDRC = 0b11111111; /* Sets all pins on Port C to output */
	PORTC = 0b00000001; /* initialize port to high – turn on leftmost LEDs */
	while (1)
	{
		pattern(&PORTC); // run the knightrider pattern continuously
	}
	
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

