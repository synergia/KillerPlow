/*
 * defines.h
 *
 *  Created on: Dec 8, 2016
 *       Authors: DefaultMan & Frozen
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include "avr/io.h"

#define max_speed 255
#define three_q_speed 192
#define half_speed 128
#define quarter_speed 64
#define min_speed 32

//Sharp's left/forward/right
#define SH_L (1<<PB3)
#define SH_F (1<<PB4)
#define SH_R (1<<PD4)

#define F_CPU 20000000UL
//TCRT's
#define TCRT1 (1<<PC0)
#define TCRT2 (1<<PC1)
#define TCRT3 (1<<PC2)
#define TCRT4 (1<<ADC7) //needed to set multiplexer

#define wb_treshold 500 // treshold for tccrts

//H_Bridges M_ == MOTOR Left/Right A/B
#define M_LA (1<<PD5)
#define M_LB (1<<PD6)
#define M_LA_OFF PORTD &= ~M_LA
#define M_LA_ON PORTD |= M_LA
#define M_LB_OFF PORTD &= ~M_LB
#define M_LB_ON PORTD |= M_LB

#define M_RA (1<<PD7) //upper connector - closer to the mosfet
#define M_RB (1<<PB0)
#define M_RA_OFF PORTD &= ~M_RA
#define M_RA_ON PORTD |= M_RA
#define M_RB_OFF PORTB &= ~M_RB
#define M_RB_ON PORTB |= M_RB

//PWM's left/right
#define PWM_L (1<<PB1)
#define PWM_R (1<<PB2)
#define PWM_R_OFF PORTB &= ~PWM_R
#define PWM_R_ON PORTB |= PWM_R
#define PWM_L_OFF PORTB &= ~PWM_L
#define PWM_L_ON PORTB |= PWM_L

//Accelerometer
#define ACMTR_INT1 (1<<PD3)
#define ACMTR_INT2 (1<<PD2)
#define ACMTR_SDA  (1<<PC4)
#define ACMTR_SCL  (1<<PC5)

#define LED_PIN (1<<PB5)
#define LED_ON PORTB &= ~LED_PIN
#define LED_OFF PORTB |= LED_PIN

typedef enum {
	left, forward, backward, right, breaking
} directions;

extern uint8_t adc_mul[5];
volatile uint16_t tccrt[5];
extern uint8_t sharp[3];

#endif /* DEFINES_H_ */
