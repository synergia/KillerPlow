/*
 * moves.h
 *
 *  Created on: Dec 8, 2016
 *       Authors: DefaultMan & Frozen
 */

#ifndef MOVES_H_
#define MOVES_H_

#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "defines.h"
#include "avr/io.h"
#include "sensors.h"

typedef struct{
	uint8_t VEL_L;
	uint8_t VEL_R;
	directions dir;
	uint8_t time;
}move;

extern  move moves[8];

void init_moves();
void invoke_pattern_move(uint8_t i); // i is move number
void set_motors_dir(directions dir);
void set_motors_vel(int vel_L, int vel_R);
void motor_soft_start(int MA_start_val, int MA_end_val, int MB_start_val,
		int MB_end_val);
void attack_enemy();
void run_from_edge();
void start_move(directions dir);

#endif /* MOVES_H_ */
