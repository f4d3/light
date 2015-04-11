
#define XTAL		16e6
#define F_CPU		16000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/pgmspace.h>


//previous pwm value:
volatile uint16_t channel0Prev = 0;
volatile uint16_t channel1Prev = 0;

//max pwm value
volatile uint16_t channel0limit = 0;
volatile uint16_t channel1limit = 0;


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
	0, 2, 8, 14, 21, 29, 38, 48, 59, 72, 95, 115, 130, 145, 170, 190, 222, 255, 300, 
	350, 430, 530, 650, 783, 916, 1204, 1582, 2079, 2732, 3590, 4717,
	 6197, 8142, 10697, 14053, 18463, 24256, 31866, 40000, 49000
	
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

uint16_t transformToOutputValue(uint8_t input)
{
	input /= encoderTicksPerMovement;
	if(input < 40)
	{
		return pgm_read_word (& pwmtable[input]);
	}
	else
	{
		return 51000;
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
	
	encode_init();

	DDRB |= ( 1 << DDB0) | ( 1 << DDB1 ) | ( 1 << DDB2 ) | ( 1 << DDB3 ) | ( 1 << DDB4 );
	DDRD |= ( 1 << DDD2) | ( 1 << DDD3) | ( 1 << DDD4) | ( 1 << DDD5) | ( 1 << DDD6) | ( 1 << DDD7);
	DDRC = 0xFF;
	DDRA = 0x00;
	
	PORTD |= (1 << PD4); // LED Off
	PORTD |= (1 << PD5); // LED Off
	
	PORTA |= (1<<PA2);

	TCCR2 |= (1 << CS21) ; //prescaler 8
	TIMSK |= (1 << TOIE2);// 8bit timer
	
	ICR1 = 0xFFFF; // 16 bit PWM
	OCR1A = 0x000; // Duty Cycle 0
	OCR1B = 0x000; // Duty Cycle 0
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1 << COM1B0));  // Disable PWM Output

	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13);
	// set Fast PWM mode using ICR1 as TOP
	
	TCCR1B |= (1 << CS10);
	// Start the timer, no prescaler
	
	
	
	sei(); // Enable interrupts

	//uart_init();
	for(;;)
	{
		
	}

	return 0;
}



ISR(TIMER2_OVF_vect)
{  //SOFTWARE PWM

	cli();

	if ( taster() )
	{
		activechannel++;
		if (activechannel == 2)
		activechannel = 0;
	}

	switch(activechannel)
	{
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

	switch(sigleds)
	{

		case 0: PORTD &= ~((1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 1: PORTD |= (1 << PD2); PORTD &= ~((1 << PD3) | (1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 2:	PORTD |= (1 << PD2); PORTD |= (1 << PD3); PORTD &= ~((1 << PD6) | (1 << PD7)); PORTC &= 0x03; break;
		case 3: PORTD |= (1 << PD2); PORTD |= (1 << PD3); PORTD |= (1 << PD6); PORTD &= ~(1 << PD7); PORTC &= 0x03; break;
		case 4: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC &= 0x03; break;
		case 5: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2); PORTC &= ~((1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 6: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3);  PORTC &= ~((1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 7: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4);  PORTC &= ~((1<<PC5) | (1<<PC6) | (1<<PC7)); break;
		case 8: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5); PORTC &= ~((1<<PC6) | (1<<PC7)); break;
		case 9: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6); PORTC &= ~(1<<PC7); break;
		case 10: PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); PORTC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC6) | (1<<PC7); break;
	}

	//Read Encoder
	int8_t new, diff;
	
	cli();
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;					// convert gray to binary
	diff = last - new;				// difference last - new
	if( diff & 1 )
	{				// bit 0 = value (1)
		last = new;					// store new as next last
		enc_delta += (diff & 2) - 1;		// bit 1 = direction (+/-)
	}
	
	
	if(channel0limit > 0 )
	{
		if(channel0Prev == 0)
		{
			// Prevents flickering when turning on led
			TCCR1B &= ~(1 << CS10);
			TCNT1 = 0xFFFE;
		}
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
		OCR1A = channel0limit;
		if(channel0Prev == 0)
		{
			TCCR1B |= (1 << CS10);
		}
	}

	else //channel0limit == 0
	{
		OCR1A = 0;
		TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
		PORTD |= (1 << PD5); // ausschalten
	}
	
	if(channel1limit > 0 )
	{
		if(channel1Prev == 0)
		{
			TCCR1B &= ~(1 << CS10);
			TCNT1 = 0xFFFE;
		}
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
		OCR1B = channel1limit;
		if(channel1Prev == 0)
		{
			TCCR1B |= (1 << CS10);
		}
	}
	else //channel1limit == 0
	{
		OCR1B = 0;
		TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
		PORTD |= (1 << PD4); // ausschalten
	}
	
	channel0Prev = channel0limit;
	channel1Prev = channel1limit;
	
	sei();
}

