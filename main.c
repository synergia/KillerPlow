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
#include "defines.h"
#include "moves.h"
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
void motor_soft_start(int MA_start_val, int MA_end_val, int MB_start_val,
		int MB_end_val);

void adc_init();
uint8_t edge_detect();
uint8_t sw_pressed();
void pwm_init();
uint8_t adc_mul[] = { 0b01000000, 0b01000001, 0b01000010, 0b01000111,
		0b01000110 };
volatile uint16_t tccrt[5];

int main() {
	init_io();
	USART_Init(__UBRR);
	adc_init();
	pwm_init();
	sei();
	uint8_t inc;

	//	if (!(PINB & SH_L)) {
	//	}
	//	if (!(PINB & SH_F)) {
	//	}
	//	if (!(PIND & SH_R)) {
	//	}

	set_motors_dir(forward);
	_delay_ms(500);
	while (!sw_pressed()) {
		uart_puts("hello");
	};

	//set_motors_vel(60,60);
	OCR1A = 60;
	OCR1B = 60;

	while (1) {
		uart_puts("\033[2J"); // clear screen
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
		if (sw_pressed())
			LED_ON;
		else
			LED_OFF;
		if (edge_detect()) {
			//stop
			uart_puts("EDGE!");
			set_motors_dir(breaking);
			OCR1A = 0;
			OCR1B = 0;
		}

		if (uart_getc() == ' ') {
			set_motors_dir(breaking);
		}

		inc++;

		_delay_ms(50);

	}

}

ISR(BADISR_vect)
{

}

ISR(ADC_vect)
{
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

	ADCSRA |= (1 << ADSC);

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

void motor_soft_start(int MA_start_val, int MA_end_val, int MB_start_val,
		int MB_end_val) {

}

/*
 * Setting motors directions
 * M_LA (1<<PB0)
 * M_LB (1<<PD7)
 * M_RA (1<<PD5)
 * M_RB (1<<PD6)
 */
void pwm_init() {
	ICR1 = 255;
	TCCR1A |=  (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);//|(1<<WGM10); // fast pwm with ICR1 as TOP
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // turn on outputs
	TCCR1B |= (1 << CS11) | (1 << CS10); // pres 64
	ICR1 = 255;
	// pwm freq = 1,2kHz
}

