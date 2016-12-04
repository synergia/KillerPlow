/*
 * main.c
 *
 *  Created on: 29.11.2016
 *      Author: frozen
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Sharp's
#define SHARP1
#define SHARP2
#define SHARP3
//Tcrt's
#define TCRT1
#define TCRT2
#define TCRT3
#define TCRT4
//H_Bridges M_ == MOTOR Left/Right
#define M_LA PD1
#define M_LB PD2
#define M_RA PD3
#define M_RB PD4

typedef enum {left,forward,backward,right} directions;

void init_io(void) {
	DDRB = 0b00000111;
	DDRC = 0b00000011;
	DDRD = 0b00010111;

	//PWM
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	//Interrupts
	MCUCR |= (1 << ISC11);
	GICR |= (1 << INT0) | (1 << INT1);

}

void soft_start(int mot_a, int mot_b){

}

/*
 * To turn to the left we have to put left motor to
 */
void set_motors(directions dir){
	switch(dir){
	case left :
		M_LA = 0;
		M_LB = 1;
		M_RA = 1;
		M_RB = 0;
		break;
	case forward :
		;
		break;
	case backward :
		;
	break;
	case right :
		;
		break;
	default :
		M_LA = 0;
		M_LB = 0;
		M_RA = 0;
		M_RB = 0;
		break;
	}
}

ISR(INT0_vect){

}
ISR(INT1_vect){

}
ISR(INT2_vect){

}

int main() {
	init_io();
	while (1) {
		if (PIND & (SHARP1<<1)){

		}

	}

}
