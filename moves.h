/*
 * moves.h
 *
 *  Created on: Dec 8, 2016
 *       Autor:
 */

#ifndef MOVES_H_
#define MOVES_H_

#include "defines.h"
#include "avr/io.h"

typedef struct{
	uint8_t PWM_A;
	uint8_t PWM_B;
	directions dir;
	uint8_t time;
}move;

extern  move moves[8];

void init_moves();
void invoke_move(uint8_t i); // i is move number
void set_motors_dir(directions dir);
void set_motors_vel(int vel_L, int vel_R);



#endif /* MOVES_H_ */
