/*
########################################################################
# MILESTONE : 4
# PROGRAM: stepperMotor.c
# PROJECT : Lab 4
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program controls a stepper motor with the MCU
########################################################################
*/
#include <stdlib.h>				// the header of the general purpose standard library of C programming language
#include <avr/io.h>				// the header of i/o port
#include <util/delay_basic.h>	// header for delay library
#include <avr/interrupt.h>		// header for interrupt function library

#define STEP1 0x30
#define STEP2 0x06
#define STEP3 0x28
#define STEP4 0x05
#define BRAKE 0x3f
#define OFF 0x00

#define TURN_30 17
#define TURN_60 33
#define TURN_90 50
#define TURN_180 100
#define DELAY 20


// global variables 
int motor_position;
int motor_steps;
int stepnum = 0;

void mTimer(int count)
{
	int i=0;
	TCCR1B |= _BV(WGM12);	// Set WGM bits to 0100, see pg 142
	OCR1A = 0x03E8;			// Set output compare register for 1000 cycles  = 1ms
	TCNT1 = 0x0000;			// Set initial value of Timer Counter to 0x000
	TIMSK1 |= 0b00000010;   // Output compare interrupt enable
	TIFR1 |= _BV(OCF1A);	// Clear timer interrupt flag and begin timer
	
	// Poll the timer to determine when the timer has reached 0x03E8 
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
	}//mTimer

void stepperClockwise(int max_steps) {
	int i;
	for(i=0;i<max_steps;i++){
		switch(stepnum){
			case(0):
				PORTA = STEP1;
				mTimer(DELAY);
				break;
			case(1):
				PORTA = STEP2;
				mTimer(DELAY);
				break;
			case(2):
				PORTA = STEP3;
				mTimer(DELAY);
				break;
			case(3):
				PORTA = STEP4;
				mTimer(DELAY);
				break;
			default:
				break;
		}//switch
		stepnum = ((stepnum+1) % 4);
		motor_steps++;
	}//for
} //stepperClockwise

void stepperWiddershins(int max_steps) {
	int i;
	for(i=0;i<max_steps;i++){
		switch(stepnum){
			case(0):
				PORTA = STEP4;
				mTimer(DELAY);
				break;
			case(1):
				PORTA = STEP3;
				mTimer(DELAY);
				break;
			case(2):
				PORTA = STEP2;
				mTimer(DELAY);
				break;
			case(3):
				PORTA = STEP1;
				mTimer(DELAY);
				break;
			default:
				break;
		}//switch
		stepnum = ((stepnum+1) % 4);
		motor_steps--;
	}//for
}//stepperWiddershins

void stepperInit(){
	stepperClockwise(TURN_90);
	motor_position = 0;
	motor_steps = 0;
}//stepperInit


/*################## MAIN ROUTINE ##################*/

int main(int argc, char *argv[]){
		
	// Set timer 1 to run at CPU clock, disable all function and use as pure timer
	TCCR1B |=  _BV(CS10);	// _BV sets the bit to logic 1, CS10 selects clock with no prescaling, f=1 MHz
		
	// Set Port A to output (for stepper motor), initialize pins to LOW
	DDRA = 0xff;
	PORTA = OFF;
	
	// Align the stepper motor
	stepperInit();
	mTimer(1000);
		
	//Turn the stepper CW 30, 60, 180 degrees CW, then the same CCW (Widdershins) with 1 sec delay in between
	stepperClockwise(TURN_30);
	mTimer(1000);
	stepperClockwise(TURN_60);
	mTimer(1000);
	stepperClockwise(TURN_180);
	mTimer(1000);
		
	stepperWiddershins(TURN_30);
	mTimer(1000);
	stepperWiddershins(TURN_60);
	mTimer(1000);
	stepperWiddershins(TURN_180);
	mTimer(1000);
		
	//This line returns a 0 value to the calling program
	// generally means no error was returned
	return (0); 
		
}//main




