/*
 * moves.c
 *
 *  Created on: Dec 8, 2016
 *       Autors: DefaultMan & Frozen
 *
 */
#include "moves.h"
move moves[8];
motors_set motors;
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
	case breaking: {
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		set_motors_vel(0, 0);
		break;
	}
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
	if (!(PINB & SH_F)) { //front
		motors.Mot_dir = forward;
		motors.Mot_A_vel = 90;
		motors.Mot_B_vel = 90;
		enemy_spotted=1;
	} else if (!(PINB & SH_L)) { //left
		motors.Mot_dir = left;
		motors.Mot_A_vel = 80;
		motors.Mot_B_vel = 80;
		enemy_spotted=1;

	} else if (!(PIND & SH_R)) { //right
		motors.Mot_dir = right;
		motors.Mot_A_vel = 80;
		motors.Mot_B_vel = 80;
		enemy_spotted=1;
	}
	else{
		enemy_spotted=0;
	}
}

void run_from_edge() {
	if (edge_detect()) {
		motors.Mot_dir = breaking;
		motors.Mot_A_vel = 0;
		motors.Mot_B_vel = 0;
	}
}

void start_move(directions dir) {
	set_motors_dir(backward);
	set_motors_vel(80, 80);
	_delay_ms(125);
	set_motors_dir(right);
	set_motors_vel(80, 80);
}

void search_enemy() {
	motors.Mot_dir = forward;
	motors.Mot_A_vel = 70;
	motors.Mot_B_vel = 70;
}

void go_to_midle_of_dojo() {
	if (tccrt[0] <= wb_treshold || tccrt[2] <= wb_treshold) {
		motors.Mot_dir = right;
		motors.Mot_A_vel = 65;
		motors.Mot_B_vel = 65;
	}
	if (tccrt[1] <= wb_treshold || tccrt[3] <= wb_treshold) {
		motors.Mot_dir = left;
		motors.Mot_A_vel = 65;
		motors.Mot_B_vel = 65;
	}
	if (tccrt[0] <= wb_treshold || tccrt[3] <= wb_treshold) {
		motors.Mot_dir = forward;
		motors.Mot_A_vel = 75;
		motors.Mot_B_vel = 75;
		time_0 =1;
	}
	if (tccrt[1] <= wb_treshold || tccrt[2] <= wb_treshold) {
		motors.Mot_dir = backward;
		motors.Mot_A_vel = 75;
		motors.Mot_B_vel = 75;
		time_0 =1;
	}

}
void start_spin() {
motors.Mot_dir = right;
motors.Mot_A_vel = 80;
motors.Mot_B_vel = 80;
}
