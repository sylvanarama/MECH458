/*
########################################################################
# MILESTONE : 5
# PROGRAM: sortingSystem.c
# PROJECT : Lab 5
# GROUP : 2
# NAME 1 : Madeleine Townley, V00752611
# NAME 2 : Alex Koszegi, V00811645
# DESC : This program implements the sorting system
########################################################################
*/

#include <stdlib.h>				// the header of the general purpose standard library of C programming language
#include <avr/io.h>				// the header of i/o port
#include <util/delay_basic.h>	// header for delay library
#include <avr/interrupt.h>		// header for interrupt function library
#include "queue.h"				// header for the linked queue function library
#include "sortingSystem.h"

// Constants
#define TRUE					1
#define FALSE					0

// States
#define CALIBRATION				1
#define OPERATIONAL				2
#define PAUSED					3
#define RAMP_DOWN				4

// MASKS
#define SENSOR_READING_MASK		0x3FF

//Button
#define PINA2_HIGH				0x04
#define PINA2_LOW				0x00	// Active

// Motor
#define CW	0x04
#define CCW	0x08
#define MOTOR_SPEED				0x70	//0XE0

// Stepper
#define STEP1 0x35
#define STEP2 0x36
#define STEP3 0x2E
#define STEP4 0x2D
#define CLOCKWISE 1
#define WIDDERSHINS -1
#define TURN_90 50
#define TURN_180 100

// Types
enum item_types {WHITE, STEEL, BLACK, ALUMINUM, TOTAL}; // to align with stepper tray order

//##############GLOBAL VARIABLES##############//

// Calibration
volatile uint16_t cal_vals_final[4][4];
//volatile uint16_t calibration_vals[4] = {897, 931, 199, 651};
//volatile uint16_t calibration_vals[4] = {720, 750, 380, 610};

// Station 4 2pm: white min, steel avg, black max, aluminum min	
volatile uint16_t calibration_vals[4] = {746, 620, 779, 438};	

//Queue
queue* itemList;
queue* entryList;
queue* reflectiveList;
queue* classifiedList;
queue* sortedList;

// ADC variables
volatile uint16_t ADC_result;
volatile uint16_t ADC_lowest_val;
volatile uint8_t reflective_present;

// Stepper variables
volatile int motor_position;
volatile int stepper_on;
uint8_t step_CW[] = {STEP1, STEP2, STEP3, STEP4};
uint8_t step_CCW[] = {STEP4, STEP3, STEP2, STEP1};
	
// Motor variables
uint8_t motor_direction = CW;

// State and Control Variables
volatile uint8_t STATE;
volatile uint8_t item_ready;
volatile uint8_t item_waiting;
volatile uint8_t item_number;
volatile uint8_t OS1_flag;
volatile uint8_t OS2_flag;
volatile uint8_t OS3_flag;
volatile uint8_t FER_flag;
volatile uint8_t ADC_flag;

//Sorted Parts: white, steel, black, aluminum, total
volatile uint8_t* sorted_items_array[5] = {0, 0, 0, 0, 0}; 
	


//##############	ISRs	##############//
ISR(TIMER0_COMPA_vect){
	// TODO: Implement ISR
	return;
}

ISR(TIMER1_COMPA_vect){
	// TODO: Implement ISR
	return;
}

// Optical Sensor 1 (PD0)
ISR(INT0_vect){
	OS1_flag = 1;
}

// Ferromagnetic Sensor (PD1)
ISR(INT1_vect){
	FER_flag = 1;
}

//Optical Sensor for ADC, edge triggered (PD2)
ISR(INT2_vect){
	OS2_flag = 1;
	if(reflective_present) reflective_present= 0;
	else reflective_present = 1;
}

// Optical sensor - exit position (PD3)
ISR(INT3_vect){
	OS3_flag = 1;
}


//Interrupt when ADC finished
ISR(ADC_vect)
{
	if(reflective_present)
	{
		ADC_result = ADCH;
		ADC_result = ((ADC_result << 8) | ADCL);
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
	// INT0 (OS1) - Falling edge
	// INT1 (Fer) - Falling edge
	// INT2 (OS2) - Either edge
	// INT3 (OS3) - Falling edge
	EICRA = 0x9A;
	
	// Enable external interrupts for Port D
	EIMSK |= 0x0F;
}

// Initialize PWM on Timer0
void init_timer0_pwm() {
	// Set Waveform Generation Mode to 3 - Fast PWM with TOP = MAX, and OCRA = Compare value
	TCCR0A |= 0x83;		// TCCR0A7:6 -> COM0A = 0b10	(inverted mode)
	// TCCR0A1:0 -> WGM1:0 = 11		(Fast PWM)
	
	// Set Clock Source
	TCCR0B |= 0x02;		// CS2:0 = 0b010 (prescaler = 8 for f_PWM = 488 Hz)
	
	// Set value we want timer to reset at (MAX)
	OCR0A = 0x80; // Duty cycle = 50%
}

// Start the motor when program starts
void init_motor() {
	PORTB =  CW;
	OCR0A = MOTOR_SPEED;
}

// Initialize the ADC when program starts
void init_ADC(){
	ADC_result = 0x3FF;
	ADC_lowest_val = 0x3FF;
	reflective_present = 0;
	item_ready = 0;
	
	// Voltage selection
	ADMUX |= _BV(REFS0);
	
	// Channel 0 gives consistent results with what's expected
	// Black has a high reflectivity and aluminum has the lowest.
	// All values are differentiable
	
	// Prescaler
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS0);
	
	// Enable Interrupt
	ADCSRA |= _BV(ADIE);
	
	// Enable ADC
	ADCSRA |= _BV(ADEN);
	
}//init_ADC

// Set Port A to output (for stepper motor), initialize pins to Step 1 position, reset global variables
void init_stepper(){
	DDRA = 0xff;
	PORTA = STEP1;
	motor_position = 1;
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

/*
int button_pressed(){
	if((PINA & WAIT) == WAIT) return 0;
	if((PINA & WAIT) != WAIT) {
		mTimer(20);
		while((PINA & WAIT) != WAIT) continue;
	}
	return 1
}//button_pressed
*/
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

void stepper_rotate(int steps, int direction) {
	int max_delay = 10;
	int min_delay = 6;
	int delay_diff = max_delay-min_delay;
	int delay = max_delay;
	static int stepnum = 0;
	int i = 0;
	for(i=0; i<steps; i++)
	{
		if(direction == CLOCKWISE)
		{
			PORTA = step_CW[stepnum];
			stepnum = (stepnum+1) % 4;
		}
		
		if(direction == WIDDERSHINS)
		{
			PORTA = step_CCW[stepnum];
			stepnum = (stepnum+1) % 4;
		}
		mTimer(delay);
		if((i<delay_diff) && (delay >= min_delay)) delay--; //acceleration
		if(((steps - i) <= delay_diff) && (delay <= max_delay)) delay++; //deceleration
	
	}//for
} //stepperRotate

void stepper_position(uint8_t new_position){
	// Brake DC motor
	PORTB = 0x00;
	
	int diff = (new_position - motor_position);
	
	if((diff == 1) || (diff == -3)) stepper_rotate(TURN_90, CLOCKWISE);
	else if((diff == -1) || (diff == 3)) stepper_rotate(TURN_90, WIDDERSHINS);
	else if((diff == 2) || (diff == -2)) stepper_rotate(TURN_180, CLOCKWISE);

	motor_position = new_position;
	
	// start DC motor
	init_motor();

}//stepper_position


void display_reflective_reading(uint16_t value) {
	// Clear upper bits in PD2 and PD5
	PORTD &= 0x0F;	// Clear PORTD LEDs and preserve sensor inputs
	
	// Get 10 bits from reading
	value &= SENSOR_READING_MASK;
	
	// Send lower 8 bits to LEDs (PORTC)
	PORTC = value;
	
	// Send bit 8 and 9 from counter to PORTD leds
	uint8_t temp = 0;
	temp = ((value & 0x100) >> 4) + ((value & 0x200) >> 2);
	
	// testing 1 - test temp
	//PORTC = temp;
	PORTD |= temp;
}

//Calibrate the ADC by running each part through the sensor 10 times, in the order: white, black, aluminum, steel
void ADC_calibrate(){
	
	int i,j,k;
	uint16_t cal_vals[10];
	uint16_t min, max, med, avg;
	
	for(j=0;j<4;j++)
	{
		// run part through 10 times, store the lowest value of each pass in an array
		for(i=0;i<10;i++)
		{
			while(!item_ready) {}
			
			// testing
			PORTC = (char)(i+1);
			//PORTC = ADC_lowest_val;
			//display_reflective_reading(ADC_lowest_val);
			
			cal_vals[i] = ADC_lowest_val;
			ADC_lowest_val = 0x3FF;
			item_ready = 0;
		}
		PORTC = 0xFF; //signal that all 10 values have been read
		
		// testing
		//update_motor_speed(0);
		PORTB = 0x00;
		
		mTimer(100);
		// calculate the minimum, maximum, median, and mean of the 10 values
		min = cal_vals[0];
		max = cal_vals[0];
		avg = cal_vals[0];
		for(k=1;k<10;k++)
		{
			if((cal_vals[k] > max) && (cal_vals[k] != 0x3FF)) max = cal_vals[k];
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
		// 1: min, 2: max, 3: med, 4: avg
		// TODO: cycle display until button pressed and then move on to next part?
		PORTC = 0x01;
		mTimer(1000);
		//PORTC = min;
		display_reflective_reading(min);
		mTimer(7000);

		PORTC = 0x02;
		mTimer(1000);
		//PORTC = max;
		display_reflective_reading(max);
		mTimer(7000);

		PORTC = 0x03;
		mTimer(1000);
		//PORTC = med;
		display_reflective_reading(med);
		mTimer(7000);

		PORTC = 0x04;
		mTimer(1000);
		//PORTC = avg;
		display_reflective_reading(avg);
		mTimer(7000);
		
		//update_motor_speed(MOTOR_SPEED);
		init_motor();
	}
}//ADC_calibrate

void entry_sensor()
{
	PORTC = 0x10;
	OS1_flag = 0;
	// To keep track of how many items have been added
	item_number++;
	//Add a new item to the queue
	item* newItem = initItem();
	newItem->number = item_number;
	newItem->stage = 1;
	enqueue(entryList, newItem);
	//PORTC = entryList->tail->number;
}

void metal_sensor(){
	//If this interrupt fires, then the object is metal
	FER_flag = 0;
	entryList->tail->metal = 1;
	PORTC |= 0x20;
}

void reflective_sensor(){
	OS2_flag = 0;
	PORTC |= 0x40;
	//object entering reflective sensor zone, start ADC conversion
	if(reflective_present)
	{
		ADCSRA |= _BV(ADSC);
	}
	// object exiting the reflective sensor zone, item ready to be classified
	else
	{
		if(STATE == OPERATIONAL)
		{
			item* reflective_sensor_item = dequeue(entryList);
			//TESTING
			/*
			next_item++;
			if(reflective_sensor_item->number != next_item) 
			{
				PORTB = 0x00;
				PORTC = next_item;
				mTimer(2000);
				PORTC = entryList->head->number;
				mTimer(2000);
			}
			*/
			reflective_sensor_item->reflective = ADC_lowest_val;
			reflective_sensor_item->stage = 2;	
			enqueue(reflectiveList, reflective_sensor_item);
			ADC_lowest_val = 0x3FF;
		}
		item_ready = 1;
	}
}

void classify_item(){
	item_ready = 0;
	item* item_to_classify = dequeue(reflectiveList);
	uint16_t r = item_to_classify->reflective;
	uint8_t m = item_to_classify->metal;
	uint8_t type = 0;
	uint16_t diff_white;
	uint16_t diff_black;
	uint16_t diff_steel;
	uint16_t diff_aluminum;

	if(m == 0)
	{
		diff_white = abs(calibration_vals[WHITE] - r);
		diff_black = abs(calibration_vals[BLACK] - r);
		if(diff_white < diff_black) 
		{
			type = WHITE;
			sorted_items_array[WHITE]++;
		}
		else 
		{
			type = BLACK;
			sorted_items_array[BLACK]++;
		}
	}
	
	if(m == 1)
	{
		diff_aluminum = abs(calibration_vals[ALUMINUM] - r);
		diff_steel = abs(calibration_vals[STEEL] - r);
		if(diff_aluminum < diff_steel) 
		{
			type = ALUMINUM;
			sorted_items_array[ALUMINUM]++;
		}
		else 
		{
			type = STEEL;
			sorted_items_array[STEEL]++;
		}
	}
	item_to_classify->type = type;
	item_to_classify->stage = 3;
	
	sorted_items_array[TOTAL]++;
	enqueue(classifiedList, item_to_classify);
	
	//TESTING
	PORTC |= item_to_classify->type;
	
}//classify_item

void exit_sensor(){
	OS3_flag = 0;
	// Show sensor triggered
	PORTC |= 0x80;
	// Move item to sorted queue
	enqueue(sortedList, dequeue(classifiedList));
}

//##############	Main Program	##############//

int main(void)
{
	// Init port directions
	DDRA = 0x00;		// Port A all inputs (button and switch)
	DDRB = 0x8F;		// PB7 = output for PWM signal, PB3:0 = output for motor
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
	init_stepper();
	sei();

	// Calibrate ADC before program starts

	//ADC_calibrate();

	entryList = initQueue();
	reflectiveList = initQueue();
	classifiedList = initQueue();
	sortedList = initQueue();

	STATE = OPERATIONAL;
	item_waiting = 0;
	item_number = 0;
	
	// Main Program
	while (1)
	{
		if(OS1_flag) entry_sensor();
		if(FER_flag) metal_sensor();
		if(OS2_flag) reflective_sensor();
		if(item_ready) classify_item();
		if(OS3_flag) 
		{
			exit_sensor();
			stepper_position((sortedList->tail->type)+1);
		}
			
	}//while
	
	clearQueue(entryList);
	clearQueue(reflectiveList);
	clearQueue(classifiedList);
	clearQueue(sortedList);
	return 0;
}//main




