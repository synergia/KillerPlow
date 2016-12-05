/*
 * main.c
 *
 *  Created on: 4 gru 2016
 *      Author: Frozen
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

//H_Bridges M_ == MOTOR Left/Right A/B
#define M_LA (1<<PB0)
#define M_LB (1<<PD7)
#define M_RA (1<<PD5)
#define M_RB (1<<PD6)

//PWM's left/right
#define PWM_L (1<<PB1)
#define PWM_R (1<<PB2)

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
typedef enum {left,forward,backward,right} directions;


void init_io(void);
void motor_soft_start(int mot_a, int mot_b);
void set_motors_dir(directions dir);
void set_motors_vel(int vel_L, int vel_R);
void adc_init();

uint8_t PROGMEM adc_mul[]={0b01000000,0b01000001,0b01000010,0b01000111};
volatile uint16_t tccrt[4];

int main() {
	init_io();
	USART_Init( __UBRR );
	adc_init();
	sei();
	uint8_t inc;
	while (1) {
		LED_ON;
		inc++;
		ADCSRA|=(1<<ADSC);
		_delay_ms(100);
		LED_OFF;

		_delay_ms(50);

		uart_puts("\033[2J");   // clear screen
		uart_puts("\033[0;0H"); // set cursor to 0,0
		uart_putint(inc,10);
		for(uint8_t j=0;j<=3;++j){
			uart_puts("TCCRTN:   ");
			uart_putint(tccrt[j],10);
			uart_puts("    \r\n");
			uart_putint(j,10);
			uart_puts("\r\n");

		}


	}


}


ISR(PCINT0_vect){
//sharp 1 2
}

ISR(PCINT2_vect){
//sharp 3
}


ISR(BADISR_vect)
{

}

ISR(ADC_vect){
	static uint8_t k;
	tccrt[k]=ADC;
	_delay_us(50);
//	ADMUX = adc_mul[k];
	++k;
	if (k>=4)k=0;

}

void adc_init(){
//	ADCSRA |=
//pres = 128 , interrupt enable enable
	ADCSRA |=(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADCSRA);
	ADCSRA |=(1<<ADEN)|(1<<ADIE);//ADSC
	ADMUX |= (1<<REFS0);
}


void init_io(void) {

	DDRB = 0b00100111;
	DDRC = 0b00000000;
	DDRD = 0b11100000;//to check

	//PWM
	//TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);//to check
	//TCCR1B |= (1 << CS10) | (1 << WGM12);//to check
	//Interrupts
	//PCMSK0 |= (1 <<PCINT0) | (1 <<PCINT1) | (1 <<PCINT2);//wrong, has to be changed
	//PCICR |= (1 << SH_L) | (1 << SH_F) | (1 << SH_R);//to check

}


void motor_soft_start(int mot_a, int mot_b){

}

/*
 * Setting motors directions
 */
void set_motors_dir(directions dir){
	switch(dir){
	case left :
		break;
	case forward :
		;
		break;
	case backward :
		;
	break;
	case right :
		;
		break;
	default :
		break;
	}
}
/*
 * Setting motors velocity
 */
void set_motors_vel(int vel_L, int vel_R){

}
