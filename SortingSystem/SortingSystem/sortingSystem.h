/*
 * sortingSystem.h
 *
 * Created: 2017-11-20 6:13:37 PM
 *  Author: AlexKoszegi
 */ 


#ifndef SORTINGSYSTEM_H_
#define SORTINGSYSTEM_H_

// Initializers
void init_interrupts();
void init_timer0_pwm();
void init_motor();
void init_ADC();
void init_stepper();

// Calibration
void ADC_calibrate();

// Timer and Buttons
void mTimer(int count);
int button_pressed();

// Motor
void update_motor_speed(uint16_t speed);
void change_motor_direction();

// Stepper
void stepper_rotate(int steps, int direction);
void stepper_position(uint8_t new_position);

// Display
void display_reflective_reading(uint16_t value);

// Classification
void classify_item();



#endif /* SORTINGSYSTEM_H_ */