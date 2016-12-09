/*
 * sensors.c
 *
 *  Created on: 9 gru 2016
 *      Authors: DefaultMan & Frozen
 */

#ifndef SENSORS_C_
#define SENSORS_C_
#include "sensors.h"

uint8_t adc_mul[5] = { 0b01000000, 0b01000001, 0b01000010, 0b01000111, 0b01000110 };
uint8_t sharp[3];

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
	sharp[0] = !(PINB & SH_L);
	sharp[1] = !(PINB & SH_F);
	sharp[2] = !(PIND & SH_R);
}

#endif /* SENSORS_C_ */
