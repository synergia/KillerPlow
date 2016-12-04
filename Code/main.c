/*
 * main.c
 *
 *  Created on: 4 gru 2016
 *      Author: Frozen
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Sharp's left/forward/right
#define SH_L PB3
#define SH_F PB4
#define SH_R PD4

//TCRT's
#define TCRT1 PC0
#define TCRT2 PC1
#define TCRT3 PC2
#define TCRT4 ADC7

//H_Bridges M_ == MOTOR Left/Right A/B
#define M_LA PB0
#define M_LB PD7
#define M_RA PD5
#define M_RB PD6

//PWM's left/right
#define PWM_L PB1
#define PWM_R PB2

//Accelerometer
#define ACMTR_INT1 PD3
#define ACMTR_INT2 PD2
#define ACMTR_SDA  PC4
#define ACMTR_SCL  PC5


typedef enum {left,forward,backward,right} directions;

void init_io(void) {
	//DDRn   76543210
	DDRB = 0b00000000;//to check
	DDRC = 0b00000000;//to check
	DDRD = 0b00000000;//to check

	//PWM
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);//to check
	TCCR1B |= (1 << CS10) | (1 << WGM12);//to check
	//Interrupts
	PCMSK0 |= (1 <<PCINT0) | (1 <<PCINT1) | (1 <<PCINT2);//wrong, has to be changed
	PCICR |= (1 << SH_L) | (1 << SH_F) | (1 << SH_R);//to check

}


void motor_soft_start(int mot_a, int mot_b){

}

/*
 * Setting motors directions
 */
void set_motors_dir(directions dir){
	switch(dir){
	case left :
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
		break;
	}
}
/*
 * Setting motors velocity
 */
void set_motors_vel(int vel_L, int vel_R){

}

ISR(PCINT0_vect){

}
ISR(PCINT1_vect){

}
ISR(PCINT2_vect){

}

int main() {
	init_io();
	while (1) {

		}
}


ISR(BADISR_vect)
{

}
