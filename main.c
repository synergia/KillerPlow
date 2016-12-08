/*
 * main.c
 *
 *  Created on: 4 gru 2016
 *      Authors: DefaultMan & Frozen
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "MKUART/mkuart.h"
//#include "I2C_TWI/i2c_twi.h"
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

#define wb_treshold 500

//H_Bridges M_ == MOTOR Left/Right A/B
#define M_LA (1<<PB0)
#define M_LB (1<<PD7)
#define M_LA_ON PORTB &= ~M_RA
#define M_LA_OFF PORTB |= M_RA
#define M_LB_ON PORTD &= ~M_RB
#define M_LB_OFF PORTD |= M_RB

#define M_RA (1<<PD5) //upper connector - closer to the mosfet
#define M_RB (1<<PD6)
#define M_RA_ON PORTD &= ~M_RA
#define M_RA_OFF PORTD |= M_RA
#define M_RB_ON PORTD &= ~M_RB
#define M_RB_OFF PORTD |= M_RB

//PWM's left/right
#define PWM_L (1<<PB1)
#define PWM_R (1<<PB2)
#define PWM_R_ON PORTB &= ~PWM_R
#define PWM_R_OFF PORTB |= PWM_R

//Accelerometer
#define ACMTR_INT1 (1<<PD3)
#define ACMTR_INT2 (1<<PD2)
#define ACMTR_SDA  (1<<PC4)
#define ACMTR_SCL  (1<<PC5)

#define LED_PIN (1<<PB5)
#define LED_ON PORTB &= ~LED_PIN
#define LED_OFF PORTB |= LED_PIN

//quote from atmega88 datasheet
/*"If any ADC [3..0] port pins are used as digital outputs,
 it is essential that these do not switch while a conversion is in progress.
 However, using the 2-wire Interface (ADC4 and ADC5) will only affect
 the conversion on ADC4 and ADC5 and not the other ADC channels"
 */

//note to LSM303D we will be using interrupts from X and Y axis sources
// X axis - fron/rear
// y axis -left/right
typedef enum {
	left, forward, backward, right
} directions;

void init_io(void);
void motor_soft_start(int mot_a, int mot_b);
void set_motors_dir(directions dir);
void set_motors_vel(int vel_L, int vel_R);
void adc_init();
uint8_t edge_detect();
uint8_t sw_pressed();
uint8_t adc_mul[] =
		{ 0b01000000, 0b01000001, 0b01000010, 0b01000111, 0b01000110 };
volatile uint16_t tccrt[5];

int main() {
	init_io();
	USART_Init( __UBRR);
	adc_init();
	sei();
	uint8_t inc;

	//set motor rotation
	M_RA_ON;
	M_RB_OFF;
	ADCSRA |= (1 << ADSC);

	if (!(PINB & SH_L)) {
	}
	if (!(PINB & SH_F)) {
	}
	if (!(PIND & SH_R)) {
	}

	while (1) {
		if (sw_pressed())
			LED_ON;
		else
			LED_OFF;

		inc++;

		_delay_ms(50);

		uart_puts("\033[2J");   // clear screen
		uart_puts("\033[0;0H"); // set cursor to 0,0
		uart_putint(inc, 10);
		uart_puts("\r\n");
		uart_putint(edge_detect(), 2);
		uart_puts("\r\n");

		for (uint8_t j = 0; j <= 3; ++j) {
			uart_puts("TCCRTN:");
			uart_putint(tccrt[j], 10);
			uart_puts("\r\n");
		}
		uart_puts("SWITCH:   ");
		uart_putint(tccrt[4], 10);
		uart_puts("    \r\n");

	}

}

ISR(BADISR_vect) {

}

ISR(ADC_vect) {
	static volatile uint8_t k;
	tccrt[k] = ADC;
	++k;
	if (k >= 5) {
		k = 0;
	}
	ADMUX = adc_mul[k];
	_delay_us(150); //there must be 13 ADC clock (84us) gap between ADMUX update and ADSC
	ADCSRA |= (1 << ADSC);
}

void adc_init() {
//	ADCSRA |=
//pres = 128 , interrupt enable enable
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADCSRA);
	ADCSRA |= (1 << ADEN) | (1 << ADIE); //ADSC
	ADMUX = (1 << REFS0);
}

void init_io(void) {

	DDRB = 0b00100111;
	DDRC = 0b00000000;
	DDRD = 0b11100000; //to check

	//PWM
	//TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);//to check
	//TCCR1B |= (1 << CS10) | (1 << WGM12);//to check
	//Interrupts
	//PCMSK0 |= (1 <<PCINT0) | (1 <<PCINT1) | (1 <<PCINT2);//wrong, has to be changed
	//PCICR |= (1 << SH_L) | (1 << SH_F) | (1 << SH_R);//to check

}

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

void motor_soft_start(int MA_start_val, int MA_end_val, int MB_start_val, int MB_end_val) {

}

/*
 * Setting motors directions
 * M_LA (1<<PB0)
 * M_LB (1<<PD7)
 * M_RA (1<<PD5)
 * M_RB (1<<PD6)
 */
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
	default: {
		break;
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
		M_LA_OFF; //  low
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
