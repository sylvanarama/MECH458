/*
########################################################################
# MILESTONE : 4
# PROGRAM: ADC.c
# PROJECT : Lab 4
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program set up an ADC channel on the MCU
########################################################################
*/

#include <stdlib.h>				
#include <avr/interrupt.h>
#include <avr/io.h>

//global variables
volatile unsigned char ADC_result;
volatile unsigned int ADC_result_flag;

/* main routine */

void main()
{
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
	
}//main

//Sensor 3: 2nd Optical Inductive, active HIGH starts ADC
/*
ISR(INT2_vect)
{
	ADCSRA |= _BV(ADSC); // ADC on rising edge
}
*/ 
//Interrupt when ADC finished
ISR(ADC_vect)
{
	ADC_result = ADCH;
	ADC_result_flag = 1;
}
