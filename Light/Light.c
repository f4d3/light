/*
PWM LED Light

Copyright (C) 2010 Christian Helm

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>
*/



#define XTAL		16e6
#define F_CPU		16000000UL




#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
//#include "cs.h"

//current pwm value:
volatile uint16_t channel0 = 0;
volatile uint16_t channel1 = 0;

//max pwm value
volatile uint16_t channel0limit = 0;
volatile uint16_t channel1limit = 0;

//pwm status
volatile uint8_t channel0state = 0;
volatile uint8_t channel1state = 0;

const uint16_t MAX_PWM = 400;

#define BAUD        9600UL
#define UBRR_BAUD   ((F_CPU/(16UL*BAUD))-1)


#define PHASE_A		!(PINA & 1<<PA0)
#define PHASE_B		!(PINA & 1<<PA1)

volatile int8_t enc_delta;			// -128 ... 127
static int8_t last;

#define TASTERPORT PINA
#define TASTERBIT PINA2

volatile uint8_t sigleds = 0;
volatile uint8_t level0 = 0;
volatile uint8_t level1 = 0;

volatile uint8_t activechannel = 0;
const uint8_t encoderTicksPerLed = 16;
const uint8_t encoderTicksPerMovement = 4;
const uint8_t ledCount = 10;


// Logarithmic brightness 
const uint16_t pwmtable[40] PROGMEM =
{
	/* calculated in open office:
	=GANZZAHL( POTENZ(2; LOG($B$2;2) * (D2+1) / $B$1) - 1)
	D2 = Input
	B2 = Number of Output Bits. Here max value = 250
	B1 = Number of Input Values = 40
	*/
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 18, 19, 20, 21,
	22, 26, 30, 35, 40, 46, 53, 61, 71, 81, 94, 108, 124, 142, 164, 188, 216, 249
	
};

uint8_t limitValue(uint8_t val)
{
		if(val > 200) // Underflow detection
		{
			val = 0;
		}
		if(val > ledCount * encoderTicksPerLed) // Limit maximum
		{
			val = ledCount * encoderTicksPerLed;
		}
		return val;
}

uint8_t transformToOutputValue(uint8_t input)
{
	input /= encoderTicksPerMovement;
	if(input < 40)
	{
		return pgm_read_word (& pwmtable[input]);
	}
	else
	{
		return 250;
	}
}

char taster(void)
{
	static unsigned char zustand;
	char rw = 0;
	
	if(zustand == 0 && !(TASTERPORT & (1<<TASTERBIT))) //Key down (rising Edge)
	{
		zustand = 1;
		rw = 1;
	}
	else if (zustand == 1 && !(TASTERPORT & (1<<TASTERBIT))) //Key pressed
	{
		zustand = 2;
		rw = 0;
	}
	else if (zustand == 2 && (TASTERPORT & (1<<TASTERBIT))) //Key up (falling Edge)
	{
		zustand = 3;
		rw = 0;
	}
	else if (zustand == 3 && (TASTERPORT & (1<<TASTERBIT)))	//Key not pressed
	{
		zustand = 0;
		rw = 0;
	}
	
	return rw;
}

int8_t encode_read1( void )			// read single step encoders
{
	int8_t val;
	
	cli();
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;					// counts since last call
}

void encode_init( void )
{
	int8_t new;
	
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;					// convert gray to binary
	last = new;					// power on state
	enc_delta = 0;
	
}

void uart_init(void)
{
	// set Baudrate
	UBRRH = (unsigned char) (UBRR_BAUD>>8);
	UBRRL = (unsigned char) (UBRR_BAUD & 0x0ff);

	//Receiver, Transmitter, Interupt
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);

	// 8 Databits, 1 Stoppbit
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}


int main(void)
{
	
	//all leds off
	PORTB |= (1 << PB0);
	PORTB |= (1 << PB1);
	PORTB |= (1 << PB2);
	PORTB |= (1 << PB3);
	PORTB |= (1 << PB4);

	encode_init();

	DDRB |= ( 1 << DDB0) | ( 1 << DDB1 ) | ( 1 << DDB2 ) | ( 1 << DDB3 ) | ( 1 << DDB4 );
	DDRD |= ( 1 << DDD4) | ( 1 << DDD5) | ( 1 << DDD6) | ( 1 << DDD7);
	DDRC = 0xFF;

	DDRA = 0x00;
	PORTA |= (1<<PA2);

	TCCR2 |= (1 << CS20) ; //prescaler 1
	TIMSK |= (1 << TOIE2);// 8bit timer

	TCCR1B |= (1 << CS11) ; //precsaler 8
	TIMSK |= (1 << TOIE1); // 16bit timer

	sei();

	//PORTD |= (1 << PD7);



	//uart_init();
	for(;;){
		
		
	}

	return 0;
}


ISR(TIMER1_OVF_vect){ //READ INPUT  //Set signal leds

cli();
	
	if ( taster() ) {
		activechannel++;
		if (activechannel == 2)
		activechannel = 0;
	}
	
	switch(activechannel){
		case 0:
		level0 += (encode_read1());
		level0 = limitValue(level0);
		channel0limit = transformToOutputValue(level0);
		sigleds = level0/encoderTicksPerLed;
		break;

		case 1:
		level1 += (encode_read1());
		level1 = limitValue(level1);
		sigleds = level1/encoderTicksPerLed;
		channel1limit = transformToOutputValue(level1);
		break;
	}

	switch(sigleds){

		case 0: PORTD &= ~((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 1: PORTD |= (1 << PD4); PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 2:	PORTD |= (1 << PD4); PORTD |= (1 << PD5); PORTD &= ~((1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 3: PORTD |= (1 << PD4); PORTD |= (1 << PD5); PORTD |= (1 << PD6); PORTD &= ~(1 << PD7); PORTC &= 0x03; break;
		case 4: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC &= 0x03; break;
		case 5: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2); PORTC &= ~((1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 6: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3);  PORTC &= ~((1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 7: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4);  PORTC &= ~((1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 8: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5); PORTC &= ~((1<<PC6) | (1<<PC7)); break;
		case 9: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6); PORTC &= ~(1<<PC7); break;
		case 10: PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7); break;
	}
	sei();

}

ISR(TIMER2_OVF_vect){  //SOFTWARE PWM

	//Read Encoder
	int8_t new, diff;
	
	cli();
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;					// convert gray to binary
	diff = last - new;				// difference last - new
	if( diff & 1 ){				// bit 0 = value (1)
		last = new;					// store new as next last
		enc_delta += (diff & 2) - 1;		// bit 1 = direction (+/-)
	}
	sei();

cli();
if(channel0limit != 0){
	if(channel0state == 0 && channel0 == (MAX_PWM - channel0limit)){
		channel0 = 0;
		channel0state = 1;
		PORTB &= ~( 1<< PB0); // einschalten !pfets!
	}
	if(channel0state == 1 && channel0 == channel0limit){
		channel0 = 0;
		channel0state = 0;
		PORTB |= (1 << PB0); // ausschalten
	}
}

if(channel1limit != 0){
	if(channel1state == 0 && channel1 == (MAX_PWM - channel1limit)){
		channel1 = 0;
		channel1state = 1;
		PORTB &= ~( 1<< PB1); // einschalten !pfets!
	}
	if(channel1state == 1 && channel1 == channel1limit){
		channel1 = 0;
		channel1state = 0;
		PORTB |= (1 << PB1); // ausschalten
	}
}

channel0++;
channel1++;


if(channel0 > MAX_PWM){
	channel0 = 0;
}
if(channel1 > MAX_PWM){
	channel1 = 0;
}
sei();

}

