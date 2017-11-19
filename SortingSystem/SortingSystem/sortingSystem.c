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
#define STEP1 0x35
#define STEP2 0x36
#define STEP3 0x2E
#define STEP4 0x2D
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
volatile uint8_t reflective_present;

// Sensor variables
volatile uint8_t is_metal;

// Stepper variables
volatile int motor_position;
volatile int stepnum;
volatile int stepper_on;

// Motor variables
uint8_t motor_direction = CW;

// State and Control Variables
volatile uint8_t STATE;
volatile uint8_t item_ready;

//##############	ISRs	##############//
ISR(TIMER0_COMPA_vect){
	// TODO: Implement ISR
	return;
}

// Optical Sensor 1 (PD0)
ISR(INT0_vect){
	// testing
	PORTC |= 0x10;
	//PORTC = PIND;
}

// Ferromagnetic Sensor (PD1)
ISR(INT1_vect){
	// testing
	PORTC |= 0x20;
	//PORTC = PIND;
}

//Optical Sensor for ADC, edge triggered (PD2)
ISR(INT2_vect){
	// testing
	PORTC |= 0x40;
	//PORTC = PIND;
	
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

// Optical sensor - exit position (PD3)
ISR(INT3_vect){
	// testing
	PORTC |= 0x80;
	//PORTC = PIND;
}

//Interrupt when ADC finished
ISR(ADC_vect)
{

	if(reflective_present) {
		//ADC_result = ((ADCH << 8) + ADCL);
		ADC_result = ADCH;
		if(ADC_result < ADC_lowest_val) ADC_lowest_val = ADC_result;
		ADCSRA |= _BV(ADSC);
	}
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
	// Specify when interrupts are triggered
		// INT0 (OS2) - Falling edge
		// INT1 (Fer) - Falling edge
		// INT2 (OS2) - Either edge
		// INT3 (OS3) - Falling edge
	EICRA = 0x9A;
	
	// Enable external interrupts for Port D
	EIMSK |= 0x0F;
}

// Initialize PWM on Timer0
void init_timer0_pwm() {
	// Set PB7 to output (for PWM signal)
	//DDRB |=0x80;		<- in main
	
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
	PORTB =  CW;
	OCR0A = MOTOR_SPEED;
}

// Initialize the ADC when program starts
void init_ADC(){	
	//initialize global variables
	ADC_result = 0xFF;
	ADC_lowest_val = 0xFF;
	reflective_present = 0;
	item_ready = 0;
	
	//configure external interrupts
	EIMSK |= (_BV(INT2)); //enable INT2
	EICRA = 0x10;
	//EICRA |= ~(_BV(ISC21) | _BV(ISC20)); //edge triggered interrupts
	
	//configure the ADC
	//by default ADC analog input set to ADC0/PORTF0
	ADCSRA |= _BV(ADEN);  //enable ADC
	ADCSRA |= _BV(ADIE);  //enable interrupts for ADC
	ADMUX  |= (_BV(ADLAR) | _BV(REFS0)); // left adjust ADC result, use AVcc as voltage ref, with ext. capacitor on AREF pin
	
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
	TCCR1B |= _BV(CS10);
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
		PORTB = 0x00;
		
		mTimer(20);
		
		// Wait until button released
		while ((PINA & 0x01) == PINA2_LOW) {}
		
		// Reverse motor direction
		if (motor_direction == CW) {
			motor_direction = CCW;
			PORTB = CCW;
			} else {
			motor_direction = CW;
			PORTB = CW;
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
}//stepper_position

void metal_sensor(queue* q){
	item* item = initItem();
	item->metal = is_metal;
	item->stage = 1;
	enqueue(q, item);
}//metal_sensor

void reflective_sensor(queue* q){
	q->head->reflective = ADC_lowest_val;
	q->head->stage = 2;
}//reflective_sensor

void exit_sensor(queue* q, char* sorted_parts[5]){
	q->head->stage = 4;
	item* item = dequeue(q);
	char type = item->type;
	deleteItem(item);
	*(sorted_parts[type])++;
}//exit_sensor

void classify_item(queue* q, uint16_t** v){
	uint16_t r = q->head->reflective;
	uint16_t m = q->head->metal;
	if((v[0][0] < r)  && (r < v[0][1]) && (m == 0)) q->head->type = 1; // white
	if((v[1][0] < r)  && (r < v[1][1]) && (m == 0)) q->head->type = 2; // black
	if((v[2][0] < r)  && (r < v[2][1]) && (m == 1)) q->head->type = 3; // aluminum
	if((v[3][0] < r)  && (r < v[3][1]) && (m == 1)) q->head->type = 4; // steel
	else q->head->type = 0; //unknown type
	q->head->stage = 3;
}//classify_part

//Calibrate the ADC by running each part through the sensor 10 times, in the order: white, black, aluminum, steel
void ADC_calibrate(uint16_t cal_vals_final[4][4]){
	
	int i,j,k;
	uint16_t cal_vals[10];
	uint16_t min, max, med, avg;
	//PORTC = 0xFF;
	
	for(j=0;j<4;j++)
	{
		// run part through 10 times, store the lowest value of each pass in an array
		for(i=0;i<10;i++)
		{
			while(!item_ready) {}
			PORTC = (char)(i+1);
			//PORTC = ADC_lowest_val;
			cal_vals[i] = ADC_lowest_val;
			ADC_lowest_val = 0xFF;
			item_ready = 0;
		}
		PORTC = 0xFF; //signal that all 10 values have been read
		mTimer(100);
		// calculate the minimum, maximum, median, and mean of the 10 values
		min = cal_vals[0];
		max = cal_vals[0];
		avg = cal_vals[0];
		for(k=1;k<10;k++)
		{
			if(cal_vals[k] > max) max = cal_vals[k];
			if(cal_vals[k] < min) min = cal_vals[k];
			avg += cal_vals[k];
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
		
		// 1: min, 2: max, 3: med, 4: avg
		// TODO: cycle display until button pressed and then move on to next part?
		PORTC = 0x01;
		mTimer(100);
		PORTC = min;
		mTimer(500);

		PORTC = 0x02;
		mTimer(100);
		PORTC = max;
		mTimer(500);

		PORTC = 0x03;
		mTimer(100);
		PORTC = med;
		mTimer(500);

		PORTC = 0x04;
		mTimer(100);
		PORTC = avg;
		mTimer(500);
	}
}//ADC_calibrate

//##############	Main Program	##############//

int main(void)
{
	// Init port directions
	DDRA = 0x00;		// Port A all inputs (button and switch)
	DDRB = 0x8F;		// PB7 = output for PWM signal
						// PB3:0 = output for motor
	DDRC = 0xFF;		// Port C all output (LEDs)
	DDRD = 0xF0;		// Port D 3:0 = sensor input (External Interrupts)
	DDRE = 0x00;		// Port E input (buttons/interrupts)
	DDRF = 0x00;		// Port F input (ADC and ?)
	
	PORTC = 0x00;
	
	// Initialize Peripherals
	cli();
	init_ADC();
	init_timer0_pwm();
	init_motor();
	init_interrupts();
	//init_stepper();
	sei();

	// Calibrate ADC before program starts
	//CHECK: is the array passed by reference? Should a struct be used instead?
	uint16_t calibration_values[4][4];
	//ADC_calibrate(calibration_values);

		
	// Main Program
	while (1)
	{
		
	}//while
	return 0;
}//main




