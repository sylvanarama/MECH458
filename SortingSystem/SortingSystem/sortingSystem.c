/*
########################################################################
# MILESTONE : 5
# PROGRAM: sortingSystem.c
# PROJECT : Lab 5
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program imprements the sorting system
########################################################################
*/

#include <stdlib.h>				// the header of the general purpose standard library of C programming language
#include <avr/io.h>				// the header of i/o port
#include <util/delay_basic.h>	// header for delay library
#include <avr/interrupt.h>		// header for interrupt function library
#include "queue.h"				// header for the linked queue function library

//Button
#define PINA2_HIGH	0x04
#define PINA2_LOW	0x00	// Active

// Motor
#define CW	0x04
#define CCW	0x08
#define MOTOR_SPEED 0x70

// Stepper
#define STEP1 0x30
#define STEP2 0x06
#define STEP3 0x28
#define STEP4 0x05
#define CLOCKWISE 1
#define WIDDERSHINS -1
#define TURN_90 50
#define TURN_180 100
#define DELAY 20

#define WAIT 0x01			// PORTx = 0bXXXXXXX1, means wait to read data from the port

//##############GLOBAL VARIABLES##############//
// ADC variables
volatile uint16_t ADC_result;
volatile uint16_t ADC_lowest_val;
volatile char reflective_present;

// Sensor variables
volatile char is_metal;

// Stepper variables
volatile int motor_position;
volatile int stepnum;
volatile int stepper_on;

// Motor variables
uint8_t motor_direction = 0x04;

// State and Control Variables
volatile char STATE;
volatile char item_ready;

//##############	ISRs	##############//
ISR(TIMER0_COMPA_vect){
	// TODO: Implement ISR
	return;
}

//Optical Sensor for ADC, edge triggered
ISR(INT2_vect){
	//object exiting reflective sensor zone, item ready to be classified
	if(reflective_present) 
	{
		reflective_present = 0;
		item_ready = 1;
	}
	// object entering the reflective sensor zone, start ADC conversion
	else
	{
		reflective_present = 1; 
		ADCSRA |= _BV(ADSC);	
	}
}

//Interrupt when ADC finished
ISR(ADC_vect)
{
	ADC_result = ((ADCH << 8) + ADCL);
	if(ADC_result < ADC_lowest_val) ADC_lowest_val = ADC_result;
	if(reflective_present) ADCSRA |= _BV(ADSC);
}

// Set up the External Interrupt 0 Vector 
ISR(INT0_vect){
	// Toggle PORTA bit 0 
	STATE = 2;
}

ISR(INT3_vect){
	// Toggle PORTA bit 3 
	STATE = 4;
}



// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
	// user code here
}

//##############	Init Functions	##############//

//Set up the interrupts
void init_interrupts(){		
	STATE = 0;
	cli();	// Disables all interrupts

	// Going to set up interrupt0 on PD0
	DDRD = 0b11110110;

	// Set up the Interrupt 0, 1, 2, 3 options
	//External Interrupt Control Register A - EICRA (pg 94 and under the EXT_INT tab to the right
	// Set Interrupt sense control to catch a rising edge or any edge
	EICRA |= _BV(ISC01) | _BV(ISC00);  //INT0: rising		  -> ?
	EICRA |= _BV(ISC11) | _BV(ISC10);  //INT1: rising		  -> ?
	EICRA |= ~_BV(ISC21) | _BV(ISC20); //INT2: edge triggered -> reflective stage reflective sensor (OR)
	EICRA |= _BV(ISC31) | _BV(ISC30);  //INT3: rising		  -> ?
	//EICRA = 0b11011111;
	

	//	EICRA &= ~_BV(ISC01) & ~_BV(ISC00); /* These lines would undo the above two lines */
	//	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */

	// See page 96 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly
	// Enable external interrupts 0-3
	EIMSK |= 0x0F;

	// Enable all interrupts
	sei();	// Note this sets the Global Enable for all interrupts
}

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
	OCR0A = MOTOR_SPEED;
}

// Initialize the ADC when program starts
void init_ADC(){
	
	//disable all interrupts
	cli();
	
	//initialize global variables
	ADC_result = 0x3F;
	ADC_lowest_val = 0x3F;
	reflective_present = 0;
	item_ready = 0;
	
	//configure external interrupts
	EIMSK |= (_BV(INT2)); //enable INT2
	EICRA |= ~(_BV(ISC21) | _BV(ISC20)); //edge triggered interrupts
	
	//configure the ADC
	//by default ADC analog input set to ADC0/PORTF0
	ADCSRA |= _BV(ADEN);  //enable ADC
	ADCSRA |= _BV(ADIE);  //enable interrupts for ADC
	ADMUX  |= (_BV(ADLAR) | _BV(REFS0)); // left adjust ADC result, use AVcc as voltage ref, with ext. capacitor on AREF pin
	
	//enable all interrupts
	sei();
	
}//init_ADC

// Set Port A to output (for stepper motor), initialize pins to Step 1 position, reset global variables
void init_stepper(){
	DDRA = 0xff;
	PORTA = STEP1;
	motor_position = 1;
	stepnum = 1;
	stepper_on = 0;
}//stepperInit

//##############	Program Functions	##############//
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

int button_pressed(){
	if((PINA & WAIT) == WAIT) return 0;
	if((PINA & WAIT) != WAIT) {
		mTimer(20);
		while((PINA & WAIT) != WAIT) continue;
	}
	return 1;
}//button_pressed

void update_motor_speed(uint16_t speed){
	OCR0A = speed;
}//update_motor_speed

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
}//change_motor_direction

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

void stepper_position(int new_position){
	int diff = (new_position - motor_position);
	
	if((diff == 1) || (diff == -3)) stepperRotate(TURN_90, CLOCKWISE);
	else if((diff == -1) || (diff == 3)) stepperRotate(TURN_90, WIDDERSHINS);
	else if((diff == 2) || (diff == -2)) stepperRotate(TURN_180, CLOCKWISE);

	motor_position = new_position;
	
	mTimer(1500);
}//stepper_position

void metal_sensor(queue* q){
	link* item = initLink();
	item->e.metal = is_metal;
	item->e.stage = 1;
	enqueue(q, item);
}//metal_sensor

void reflective_sensor(queue* q){
	q->head->e.reflective = ADC_lowest_val;
	q->head->e.stage = 2;
}//reflective_sensor

void exit_sensor(queue* q, char* sorted_parts[5]){
	q->head->e.stage = 4;
	link* item = dequeue(q);
	int type = item->e.type;
	deleteLink(item);
	*(sorted_parts[type])++;
}//exit_sensor

void classify_item(queue* q, uint16_t** v){
	uint16_t r = q->head->e.reflective;
	int m = q->head->e.metal;
	if((v[0][0] < r)  && (r < v[0][1]) && (m == 0)) q->head->e.type = 1; // white
	if((v[1][0] < r)  && (r < v[1][1]) && (m == 0)) q->head->e.type = 2; // black
	if((v[2][0] < r)  && (r < v[2][1]) && (m == 1)) q->head->e.type = 3; // aluminum
	if((v[3][0] < r)  && (r < v[3][1]) && (m == 1)) q->head->e.type = 4; // steel
	else q->head->e.type = 0; //unknown type
	q->head->e.stage = 3;
}//classify_part

//Calibrate the ADC by running each part through the sensor 10 times, in the order: white, black, aluminum, steel
void ADC_calibrate(uint16_t cal_vals_final[4][4]){
	init_ADC();
	init_timer0_pwm();
	init_motor();
	int i,j,k;
	uint16_t cal_vals[10];
	uint16_t min, max, med, avg;
	PORTC = 0xFF;
	
	for(j=0;j<1;j++)
	{
		// run part through 10 times, store the lowest value of each pass in an array
		for(i=0;i<10;i++)
		{
			while(!item_ready) {}
			PORTC = (char)i;
			cal_vals[i] = ADC_lowest_val;	
			ADC_lowest_val = 0x3F;	
			item_ready = 0;		
		}
		update_motor_speed(0x00);
		// calculate the minimum, maximum, median, and mean of the 10 values
		min = cal_vals[0];
		max = cal_vals[0];
		avg = cal_vals[0];
		for(k=1;k<10;k++)
		{
			if(cal_vals[i] > max) max = cal_vals[i];
			if(cal_vals[i] < min) min = cal_vals[i];
			avg += cal_vals[i];
		}
		med = (min+max)/2;
		avg = avg/10;
		
		//store the results in a 2D array: 
		//           min  max  med  avg
		// white    [0,0][0,1][0,2][0,3]
		// black    [1,0][1,1][1,2][1,3]
		// aluminum [2,0][2,1][2,2][2,3]
		// steel    [3,0][3,1][3,2][3,3]
		
		cal_vals_final[j][0] = min;
		cal_vals_final[j][1] = max;
		cal_vals_final[j][2] = med;
		cal_vals_final[j][3] = avg;
		
		// display the results for the part
		PORTC = j;
		mTimer(1000);
		PORTC = min & 0xFF00;
		mTimer(1000);
		PORTC = max & 0xFF00;
		mTimer(1000);
		PORTC = med & 0xFF00;
		mTimer(1000);
		PORTC = avg & 0xFF00;
		mTimer(1000);
	//}	
}//ADC_calibrate

//##############	Main Program	##############//

int main(void)
{
	// Init port directions
	DDRA = 0x00;		// Port A all inputs (button and switch)
	DDRC = 0xFF;		// Port C all output (LEDs)
	DDRD = 0xFF;		// Port D 3:0 = output (Motor)
	
	// Calibrate ADC before program starts
	uint16_t calibration_values[4][4];
	ADC_calibrate(calibration_values);
	
	// Init peripherals
	//queue* itemList = initQueue();
	//init_interrupts();
	//init_timer0_pwm();
	//init_ADC();	
	//init_stepper();
	//init_motor();
	
	// Main Program
	while (1)
	{
		/*
		goto POLLING_STAGE;

		// POLLING STATE
		POLLING_STAGE:
		PORTC = 0x0F;	// Indicates this state is active
		switch(STATE){
			case (0) :
				goto POLLING_STAGE;
				break;	
			case (1) :
				goto MAGNETIC_STAGE;
				break;
			case (2) :
				goto REFLECTIVE_STAGE;
				break;
			case (4) :
				goto BUCKET_STAGE;
				break;
			case (5) :
				goto END;
			default :
				goto POLLING_STAGE;
		}//switch STATE
		

		MAGNETIC_STAGE:
		// Do whatever is necessary HERE
		PORTC = 0x01; // Just output pretty lights know you made it here
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;

		REFLECTIVE_STAGE:
		// Do whatever is necessary HERE
		PORTC = 0x02; // Just output pretty lights know you made it here
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;
		
		BUCKET_STAGE:
		// Do whatever is necessary HERE
		PORTC = 0x04;
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;
		
		END:
		// The closing STATE ... how would you get here?
		PORTC = 0xF0;	// Indicates this state is active
		// Stop everything here...'MAKE SAFE'
		*/
	}//while
	return 0;
}//main


