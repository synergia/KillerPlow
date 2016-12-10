/*
 * main.c
 *
 *  Created on: 4 gru 2016
 *      Authors: DefaultMan & Frozen
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "MKUART/mkuart.h"
#include "defines.h"
#include "moves.h"
#include "sensors.h"
//#include "I2C_TWI/i2c_twi.h"

//quote from atmega88 datasheet
/*"If any ADC [3..0] port pins are used as digital outputs,
 it is essential that these do not switch while a conversion is in progress.
 However, using the 2-wire Interface (ADC4 and ADC5) will only affect
 the conversion on ADC4 and ADC5 and not the other ADC channels"
 */

//note to LSM303D we will be using interrupts from X and Y axis sources
// X axis - fron/rear
// y axis -left/right
void init_io(void);
void adc_init();
void pwm_init();
void tim_init();

//void tim_init();
volatile int counter = 0;
int main() {
	init_io();
	USART_Init(__UBRR);
	adc_init();
	pwm_init();
	tim_init();

	sei();

	while (!sw_pressed()) {
		//	ADCSRA |= (1 << ADSC);
//		PORTB ^= LED_PIN;
//		_delay_ms(100);
		if (edge_detect()) {
			LED_ON;
		} else {
			LED_OFF;
		}
	};
	LED_OFF;
	_delay_ms(3000);
	start_move(left);
	while (1) {
		//	ADCSRA |= (1 << ADSC);
		attack_enemy();
		run_from_edge();

		set_motors_dir(motors.Mot_dir);
		set_motors_vel(motors.Mot_A_vel, motors.Mot_B_vel);
//		if (uart_getc() == 'w') {
//			set_motors_dir(forward);
//			set_motors_vel(motors.Mot_A_vel, motors.Mot_B_vel);
//			_delay_ms(100);
//			set_motors_dir(breaking);
//			uart_putc('v');
//			;
//		}
//
//		if (uart_getc() == 's') {
//			set_motors_dir(backward);
//			set_motors_vel(motors.Mot_A_vel, motors.Mot_B_vel);
//			_delay_ms(100);
//			set_motors_dir(breaking);
//			;
//		}
//
//		if (uart_getc() == 'a') {
//			set_motors_dir(left);
//			set_motors_vel(motors.Mot_A_vel, motors.Mot_B_vel);
//			_delay_ms(100);
//			set_motors_dir(breaking);
//			;
//		}
//
//		if (uart_getc() == 'd') {
//			set_motors_dir(right);
//			set_motors_vel(motors.Mot_A_vel, motors.Mot_B_vel);
//			_delay_ms(100);
//			set_motors_dir(breaking);
//			;
//		}
//		uart_puts("\033[2J"); // clear screen
//		uart_puts("\033[0;0H"); // set cursor to 0,0
//		uart_puts("\r\n");
//		uart_putint(!(PINB & SH_L), 2);
//		uart_puts("\r\n");
//		uart_putint(!(PINB & SH_F), 2);
//		uart_puts("\r\n");
//		uart_putint(!(PIND & SH_R), 2);
//		uart_puts("\r\n");

//		uart_puts("\033[2J"); // clear screen
//		uart_puts("\033[0;0H"); // set cursor to 0,0
//		uart_puts("\r\n");
//		uart_putint(edge_detect(), 2);
//		uart_puts("\r\n");
//		/*TCCRTs debug info*/
//		for (uint8_t j = 0; j <= 3; ++j) {
//			uart_puts("TCCRTN:");
//			uart_putint(tccrt[j], 10);
//			uart_puts("\r\n");
//		}

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
}

ISR(TIMER0_COMPA_vect) {
	static uint16_t x;
	++x;
	if (x >= 1000) {
	//	PORTB ^= LED_PIN;
		x = 0;
	}
}

void adc_init() {
	//	ADCSRA |=
	//pres = 128 , interrupt enable enable
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADCSRA);
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADATE); //ADSC
	ADMUX = (1 << REFS0);
	ADCSRB |= (1 << ADTS1) | (1 << ADTS0);
	_delay_us(250);
//	ADCSRA |= (1 << ADSC);
	for (uint8_t i = 0; i <= 4; ++i) {
		tccrt[i] = 1023;
	}

}

void init_io(void) {

	DDRB = 0b00100111;
	DDRC = 0b00000000;
	DDRD = 0b11100000;
}

void tim_init() {
	TCCR0A |= (1 << WGM01); //CTC mode
	TCCR0B |= (1 << CS02) | (1 << CS00); //CLK, 1024 prescaler
	TCNT0 = 0;
	TIMSK0 |= (1 << OCIE0A); //enable interrupts
	OCR0A = 19; //1ms interrupt
}

void pwm_init() {
	/*Timer1*/
	ICR1 = 255;
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12); //|(1<<WGM10); // fast pwm with ICR1 as TOP
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // turn on outputs
	TCCR1B |= (1 << CS11) | (1 << CS10); // pres 64
	ICR1 = 255;
	// pwm freq = 1,2kHz

}

