/*
 * moves.c
 *
 *  Created on: Dec 8, 2016
 *       Autor:
 *
 */
#include "moves.h"

move moves[8];

void init_moves(){

	moves[0].PWM_A = half_speed;
	moves[0].PWM_B = half_speed;
	moves[0].dir = forward;
	moves[0].time = 200;

	moves[1].PWM_A = half_speed;
	moves[1].PWM_B = half_speed;
	moves[1].dir = left;
	moves[1].time = 200;

	moves[2].PWM_A = half_speed;
	moves[2].PWM_B = half_speed;
	moves[2].dir = right;
	moves[2].time = 200;

	moves[3].PWM_A = half_speed;
	moves[3].PWM_B = half_speed;
	moves[3].dir = backward;
	moves[3].time = 200;

	moves[4].PWM_A = half_speed;
	moves[4].PWM_B = quarter_speed;
	moves[4].dir = forward;
	moves[4].time = 200;

	moves[5].PWM_A = quarter_speed;
	moves[5].PWM_B = half_speed;
	moves[5].dir = forward;
	moves[5].time = 200;

	moves[6].PWM_A = quarter_speed;
	moves[6].PWM_B = half_speed;
	moves[6].dir = backward;
	moves[6].time = 200;

	moves[7].PWM_A = half_speed;
	moves[7].PWM_B = quarter_speed;
	moves[7].dir = backward;
	moves[7].time = 200;
}

void invoke_move(uint8_t i){
	set_motors_vel(moves[i].PWM_A,moves[i].PWM_B );
	set_motors_dir(moves[i].dir);
//time will be set in timer interrupt
}

void set_motors_dir(directions dir) {
	switch (dir) {
	case left: {
		M_LA_OFF; //  low
		M_LB_ON; //  high
		M_RA_ON; //  high
		M_RB_OFF; //  low
		break;
	}
	case forward: {
		M_LA_ON; //  high
		M_LB_OFF; //  low
		M_RA_ON; //  high
		M_RB_OFF; //  low
		break;
	}

	case backward: {
		M_LA_OFF; //  low
		M_LB_ON; //  high
		M_RA_OFF; //  low
		M_RB_ON; //  high
		break;
	}
	case right: {
		M_LA_ON; //  high
		M_LB_OFF; //  low
		M_RA_OFF; //  low
		M_RB_ON; //  high
		break;
	}
	case breaking :
	default: {
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		break;
	}
	}
}
/*
 * Setting motors velocity
 * vel_N are in % from 0-100
 * pwm_N are int's from 0-255
 */
void set_motors_vel(int vel_L, int vel_R) {
	if (abs(OCR1A - vel_L) > 50 | abs(OCR1B - vel_R) > 50) {
		motor_soft_start(OCR1A, vel_L, OCR1B, vel_R);
	} else {
		OCR1A = (int) 2.55 * vel_L;
		OCR1B = (int) 2.55 * vel_R;
	}
}
