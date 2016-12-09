/*
 * moves.c
 *
 *  Created on: Dec 8, 2016
 *       Autors: DefaultMan & Frozen
 *
 */
#include "moves.h"
move moves[8];
/* Listy of basic moves that robot can make*/
void init_moves() {

	moves[0].VEL_L = half_speed;
	moves[0].VEL_R = half_speed;
	moves[0].dir = forward;
	moves[0].time = 200;

	moves[1].VEL_L = half_speed;
	moves[1].VEL_R = half_speed;
	moves[1].dir = left;
	moves[1].time = 200;

	moves[2].VEL_L = half_speed;
	moves[2].VEL_R = half_speed;
	moves[2].dir = right;
	moves[2].time = 200;

	moves[3].VEL_L = half_speed;
	moves[3].VEL_R = half_speed;
	moves[3].dir = backward;
	moves[3].time = 200;

	moves[4].VEL_L = half_speed;
	moves[4].VEL_R = quarter_speed;
	moves[4].dir = forward;
	moves[4].time = 200;

	moves[5].VEL_L = quarter_speed;
	moves[5].VEL_R = half_speed;
	moves[5].dir = forward;
	moves[5].time = 200;

	moves[6].VEL_L = quarter_speed;
	moves[6].VEL_R = half_speed;
	moves[6].dir = backward;
	moves[6].time = 200;

	moves[7].VEL_L = half_speed;
	moves[7].VEL_R = quarter_speed;
	moves[7].dir = backward;
	moves[7].time = 200;
}

void invoke_pattern_move(uint8_t i) {
	set_motors_vel(moves[i].VEL_L, moves[i].VEL_R);
	set_motors_dir(moves[i].dir);
//time will be set in timer interrupt
}

/*if we find time*/
void motor_soft_start(int MA_start_val, int MA_end_val, int MB_start_val,
		int MB_end_val) {

}

void set_motors_dir(directions dir) {
	switch (dir) {
	case right: {
		M_LA_OFF; //  low
		M_LB_ON; //  high
		M_RA_ON; //  high
		M_RB_OFF; //  low
		break;
	}
	case backward: {
		M_LA_ON; //  high
		M_LB_OFF; //  low
		M_RA_ON; //  high
		M_RB_OFF; //  low
		break;
	}

	case forward: {
		M_LA_OFF; //  low
		M_LB_ON; //  high
		M_RA_OFF; //  low
		M_RB_ON; //  high
		break;
	}
	case left: {
		M_LA_ON; //  high
		M_LB_OFF; //  low
		M_RA_OFF; //  low
		M_RB_ON; //  high
		break;
	}
	case breaking:
	default: {
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		break;
	}
	}
}

void set_motors_vel(int vel_L, int vel_R) {
	if (vel_L < 256 || vel_R < 256) {
		OCR1A = (int) 2.55 * vel_L;
		OCR1B = (int) 2.55 * vel_R;
	} else {
		OCR1A = 0;
		OCR1B = 0;
	}

}

void attack_enemy() {
	if (sharp[0]) { //left
		set_motors_dir(left);
		set_motors_vel(50, 50);
	}
	if (sharp[1]) { //front
		set_motors_dir(forward);
		set_motors_vel(80, 80);
	}
	if (sharp[2]) { //right
		set_motors_dir(right);
		set_motors_vel(50, 50);
	}
}

void run_from_edge() {

}

void start_move(directions dir) {
	set_motors_dir(backward);
	set_motors_vel(80, 80);
	_delay_ms(250);
	if(dir == left)
		set_motors_dir(left);
	else
		set_motors_dir(right);
	set_motors_vel(80, 80);
	_delay_ms(500);
	set_motors_dir(breaking);
	set_motors_vel(0, 0);
}
