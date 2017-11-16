/*
########################################################################
# MILESTONE : 4
# PROGRAM: stepper_and_ADC.c
# PROJECT : Lab 4
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program set up an ADC channel on the MCU, and controls the stepper motor
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
#define CLOCKWISE 1
#define WIDDERSHINS -1
#define TURN_90 50
#define TURN_180 100

//##############GLOBAL VARIABLES##############//
volatile unsigned char ADC_result;
volatile unsigned int ADC_result_flag;
volatile int motor_position;
volatile int stepnum;
volatile int stepper_on;

//#############FUNCTIONS###################//

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

void stepperRotate(int steps, int direction) {
	stepper_on = 1;
	int delay = 20;
	int i;
	for(i=0;i<steps;i++){
		if(direction == CLOCKWISE)   stepnum = ((stepnum % 4) + 1); 
		if(direction == WIDDERSHINS) stepnum = ((stepnum - 1) % 4);
		if(stepnum == 0) stepnum = 4; 
		switch(stepnum){
			case(1):
				PORTA = STEP1;
				mTimer(delay);
				break;
			case(2):
				PORTA = STEP2;
				mTimer(delay);
				break;
			case(3):
				PORTA = STEP3;
				mTimer(delay);
				break;
			case(4):
				PORTA = STEP4;
				mTimer(delay);
				break;
			default: break;
		}//switch
		if((i<5) && (delay >= 10)) delay -= 2; //acceleration
		if(((steps - i) <= 5) && (delay <=20)) delay += 2; //deceleration
	}//for
	stepper_on = 0;
} //stepperRotate


// Set Port A to output (for stepper motor), initialize pins to Step 1 position, reset global variables
void stepperInit(){
	DDRA = 0xff;
	PORTA = STEP1;
	motor_position = 1;
	stepnum = 1;
	stepper_on = 0;
}//stepperInit

void stepper_move(int new_position){
	int diff = (new_position - motor_position);
	
	if((diff == 1) || (diff == -3)) stepperRotate(TURN_90, CLOCKWISE);
	else if((diff == -1) || (diff == 3)) stepperRotate(TURN_90, WIDDERSHINS);
	else if((diff == 2) || (diff == -2)) stepperRotate(TURN_180, CLOCKWISE);

	motor_position = new_position;
	
	mTimer(1500);
}//stepper_move

void ADC_driver(){
	//disable all interrupts
	cli();
		
	//configure external interrupts
	EIMSK |= (_BV(INT2)); //enable INT2
	EICRA |= (_BV(ISC21) | _BV(ISC20)); //rising edge interrupt
		
	//configure the ADC
	//by default ADC analog input set to ADC0/PORTF0
	ADCSRA |= _BV(ADEN);  //enable ADC
	ADCSRA |= _BV(ADIE);  //enable interrupts for ADC
	ADMUX  |= (_BV(ADLAR) | _BV(REFS0)); // left adjust ADC result, use AVcc as voltage ref, with ext. capacitor on AREF pin
		
	//set PORTC as output to display the ADC result, set to LOW initially
	DDRC = 0xff;
	PORTC = 0x00;
		
	//enable all interrupts
	sei();
		
	//initialize ADC, start one conversion at the beginning
	ADCSRA |= _BV(ADSC);
	while(1){
		if(ADC_result_flag){
			PORTC = ADC_result;
			ADC_result_flag = 0x00;
			ADCSRA |= _BV(ADSC); // ADC on rising edge
		}//if
	}//while
}//ADC_driver

//Interrupt for when ADC is finished
ISR(ADC_vect)
{
	ADC_result = ADCH;
	ADC_result_flag = 1;
}

int main(){
	//ADC_driver();
	TCCR1B |=  _BV(CS10);
	DDRC = 0xFF;
	PORTC = 0x00;
	stepperInit();
	stepper_move(1);
	stepper_move(2);
	stepper_move(1);
	stepper_move(3);
	stepper_move(1);
	stepper_move(4);

	return 0;
}