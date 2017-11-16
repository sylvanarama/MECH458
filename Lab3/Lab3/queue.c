/*
########################################################################
# MILESTONE : 3
# PROGRAM : queue.c
# PROJECT : Lab 3
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program reads data from a port on the MCU, places that data into a queue, then outputs it for display
########################################################################
*/
#include <stdlib.h>				// the header of the general purpose standard library of C programming language
#include <avr/io.h>				// the header of i/o port
#include <util/delay_basic.h>	// header for delay library
#include <avr/interrupt.h>		// header for interrupt function library
#include "LinkedQueue.h"		// header for the linked queue function library

#define WAIT 0x04			// PORTx = 0bXXXXX1XX, means wait to read data from the port 

void mTimer(int count)
{
	int i=0;
	TCCR1B |= _BV(WGM12);	// Set WGM bits to 0100, see pg 142
	OCR1A = 0x03E8;			// Set output compare register for 1000 cycles  = 1ms
	TCNT1 = 0x0000;			// Set initial value of Timer Counter to 0x000
	TIMSK1 |= 0b00000010;   // Output compare interrupt enable 
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

/*################## MAIN ROUTINE ##################*/

int main(int argc, char *argv[]){
	/*
	// Set timer 1 to run at CPU clock, disable all function and use as pure timer 
	TCCR1B |=  _BV(CS10);	// _BV sets the bit to logic 1, CS10 selects clock with no prescaling, f=1 MHz
	
	// Configure I/O ports 
	DDRC = 0xff;			//Set all pins on Port C to output 
	PORTC = 0x00;			//Set all pins on Port C to LOW
	DDRA = 0x00;			//Set all pins on Port A to input
	PORTA = 0x00;			//Disable pull-up resistors to set unconnected pins to LOW

	
	// Set up a linked list to hold the data read from Port A 
	link *head;			// The ptr to the head of the queue 
	link *tail;			// The ptr to the tail of the queue 
	link *newLink;		// A ptr to a link aggregate data type (struct) 
	link *rtnLink;		// same as the above 
	
	rtnLink = NULL;
	newLink = NULL;

	setup(&head, &tail);			
	
	// Read data and add to queue, until the list is 4 items long
	uint8_t input = PINA;
	char item;
	char display;
	int num_presses = 0;
	
	// loop for entire program
	while(1)
	{
		// wait until button pressed a fifth time
		if(((PINA & WAIT) == WAIT) && (num_presses == 4)) continue;
		
		// reset when button pressed for fifth time
		if(((PINA & WAIT) != WAIT) && (num_presses == 4))
		{
			while((PINA & WAIT) != WAIT) continue; // when button pressed, wait until released
			mTimer(20); // delay to account for contact bounce
			PORTC = 0x00;
			num_presses = 0;
			clearQueue(&head, &tail);
		}
	
		//loop while acquiring data, up to four items
		while(num_presses < 4)
		{
			if((PINA & WAIT) == WAIT) continue; // continue to next iteration of loop if button not pressed 
			while((PINA & WAIT) != WAIT) continue; // button pressed, wait until released
			mTimer(20); // delay to account for contact bounce
			input = PINA; // read data from Port A
			input &= 0x03; // clear all bits but the last two
			
			// Initialize new link in the queue with data read from PINA
			initLink(&newLink);
			newLink->e.stage = input;
			enqueue(&head, &tail, &newLink);
			num_presses++;
		}
		
		// output data in the queue to PORTC 
		
		//discard first item in the queue
		dequeue(&head, &tail, &rtnLink);
		free(rtnLink);
	
		//display item 1
		dequeue(&head, &head, &rtnLink);
		item = rtnLink->e.stage;
		free(rtnLink);
		display = item;
		PORTC = display;
		mTimer(2000);
	
		//display items 1 and 2
		dequeue(&head, &tail, &rtnLink);
		item = rtnLink->e.stage;
		free(rtnLink);
		display = ((item << 2) | display);
		PORTC = display;
		mTimer(2000);
	
		//display items 1, 2, and 3
		dequeue(&head, &tail, &rtnLink);
		item = rtnLink->e.stage;
		free(rtnLink);
		display = ((item << 4) | display);
		PORTC = display;
		mTimer(2000);

}
	*/
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}




