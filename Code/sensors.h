/*
 * sensors.h
 *
 *  Created on: 9 gru 2016
 *      Authors: DefaultMan & Frozen
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "defines.h"
#include "avr/io.h"

uint8_t sw_pressed();
uint8_t edge_detect();
void enemy_detect();

#endif /* SENSORS_H_ */
