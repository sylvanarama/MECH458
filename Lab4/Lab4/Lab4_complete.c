/*
########################################################################
# MILESTONE : 4A - PWM Component
# PROGRAM : 4
# PROJECT : Lab 4
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program establishes a PWM output signal on Port B Pin 7 at
			488 Hz with a 50% duty cycle, set up an ADC channel on the MCU, 
			and controls the stepper motor
# DATA
# REVISED
########################################################################
*/ 

#include <stdlib.h>				// the header of the general purpose standard library of C programming language
#include <util/delay_basic.h>	// header for delay library
#include <avr/io.h>
#include <avr/interrupt.h>

// Stepper
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

//Button
#define PINA2_HIGH	0x04
#define PINA2_LOW	0x00	// Active

// Motor
#define CW	0x04
#define CCW	0x08

//##############GLOBAL VARIABLES##############//

// ADC variables
volatile unsigned char ADC_result;
volatile unsigned int ADC_result_flag;

// Stepper variables
int motor_position;
int motor_steps;
int stepnum = 0;

// Motor variables
uint8_t motor_direction = 0x04;

//##############	ISRs	##############//
ISR(TIMER0_COMPA_vect){
	// TODO: Implement ISR
	return;
}

//Interrupt when ADC finished
ISR(ADC_vect)
{
	ADC_result = ADCH;
	ADC_result_flag = 1;
}

//##############	Init Functions	##############//

// Initialize PWM on Timer0
void init_timer0_pwm() {
	// Set PB7 to output (for PWM signal)
	DDRB |=0x80;		
	
	// Set Waveform Generation Mode to 3 - Fast PWM with TOP = MAX, and OCRA = Compare value
	TCCR0A |= 0x83;		// TCCR0A7:6 -> COM0A = 0b10	(inverted mode)
						// TCCR0A1:0 -> WGM1:0 = 11		(Fast PWM)
	
	// Set Clock Source 
	TCCR0B |= 0x02;		// CS2:0 = 0b010 (prescaler = 8 for f_PWM = 488 Hz)
	
	// Set value we want timer to reset at (MAX)
	OCR0A = 0x80; // Duty cycle = 50% 
	
	// Enable Output Compare A Interrupt
	//TIMSK0 |= 0x01;
	
	// Enable global interrupts
	//sei();
}

// Start the motor when program starts
void init_motor() {
	PORTD =  CW;
}

void init_ADC(){
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
	PORTC = 0x00;
	
	//enable all interrupts
	sei();
	
}//ADC

//##############	Program Functions	##############//
void update_motor_speed(uint16_t speed){
	OCR0A = speed;
}

void change_motor_direction() {
	if ((PINA & 0x01) == PINA2_LOW) {
		// Break the motor
		PORTD = 0x00;
		
		mTimer(20);
		
		// Wait until button released
		while ((PINA & 0x01) == PINA2_LOW) {}
		
		// Reverse motor direction
		if (motor_direction == CW) {
			motor_direction = CCW;
			PORTD = CCW;
			} else {
			motor_direction = CW;
			PORTD = CW;
		}
	}
}

void ADC_run() {
	//initialize ADC, start one conversion at the beginning
	ADCSRA |= _BV(ADSC);
	
	if(ADC_result_flag){
			PORTC = ADC_result;
			update_motor_speed(ADC_result);
			ADC_result_flag = 0x00;
			ADCSRA |= _BV(ADSC); // ADC on rising edge
		}//if
}

void mTimer(int count)
{
	int i=0;
	TCCR3B |=  _BV(CS10);
	TCCR3B |= _BV(WGM12);	// Set WGM bits to 0100, see pg 142
	OCR3A = 0x03E8;			// Set output compare register for 1000 cycles  = 1ms
	TCNT3 = 0x0000;			// Set initial value of Timer Counter to 0x000
	//TIMSK3 |= 0b00000010;   // Output compare interrupt enable --remove??
	TIFR3 |= _BV(OCF3A);	// Clear timer interrupt flag and begin timer
	
	// Poll the timer to determine when the timer has reached 0x03E8 
	while(i<count)
	{
		if((TIFR3 & 0x02) == 0x02)
		{
			// clear interrupt flag by writing a 1 to the bit
			TIFR3 |= _BV(OCR3A);
			i++;
		}
	}
	
	return;
	} /*mTimer*/

//##############	Main Program	##############//

int main(void)
{
	// Init port directions
	DDRA = 0x00;		// Port A all inputs (button and switch)
	DDRC = 0xFF;		// Port C all output (LEDs)
	DDRD = 0xFF;		// Port D 3:0 = output (Motor)
	
	// Init peripherals
	init_motor();
	init_timer0_pwm();
	init_ADC();
	
	// Main Program
    while (1) 
    {
		// Poll ADC
		ADC_run();
		
		// Check for button press
		change_motor_direction();
    }
}

