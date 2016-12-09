/*
 * sensors.c
 *
 *  Created on: 9 gru 2016
 *      Authors: DefaultMan & Frozen
 */

#ifndef SENSORS_C_
#define SENSORS_C_
#include "sensors.h"

uint8_t sw_pressed() {
	if (tccrt[4] <= 100)
		return 1;
	else
		return 0;

}

uint8_t edge_detect() {
	uint8_t x = 0;

	for (uint8_t i = 0; i <= 3; ++i) {
		if (tccrt[i] < wb_treshold)
			x |= (1 << i); //set bit if edge detected
		//	eg. 	0000 0011 edge on one side
		// eg.		0000 1100 edge on the oposite side
		// eg. 		0000 0100 edge on the robot's corner
	}

	return x;
}

void enemy_detect() {
	if (!(PORTB & SH_L)) {
		sharp[0] = 1;
	} else {
		sharp[0] = 0;
	}
	if (!(PORTB & SH_F)) {
		sharp[1] = 1;
	} else {
		sharp[1] = 0;
	}
	if (!(PORTD & SH_R)) {
		sharp[2] = 1;
	} else {
		sharp[2] = 0;
	}
}

#endif /* SENSORS_C_ */
