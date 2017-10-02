/*
 * Lab1P1.c
########################################################################
# MILESTONE : 1
# PROGRAM : 1
# PROJECT : MyFirstProject
# GROUP : X
# NAME 1 : Madeleine Townley V00752611
# NAME 2 : First Name, Last Name, Student ID
# DESC : This program does…[write out a brief summary]
# DATA
# REVISED
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
/*################## MAIN ROUTINE ##################*/
int main(int argc, char *argv[]){
	DDRD = 0b11111111; /* Sets all pins on Port D to output */
	PORTD = 0b00011100; /* initialize port to high – turn on LEDs */
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

