// Hardware abstraction layer for 2nd hw revision

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUD 115200
#include <util/setbaud.h>

#include "hal2.h"
#include "ovi.h"

// Mechanical constants
int const MAIN_MOTOR_PWM_TOP = 255;
int const MAIN_MOTOR_BRAKE_SPEED = 120;
int const MAIN_MOTOR_MIN_SPEED = 150;
int const MAIN_MOTOR_MED_SPEED = 254;
int const MAIN_MOTOR_MAX_SPEED = 255;
int const MAGNET_OPEN_WAIT = 5;		// 10ths of a second

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

#define CUR_LIMIT_PWM_1 OCR3A
#define CUR_LIMIT_PWM_2 OCR3B
#define PWM_1  OCR0
#define PWM_2  OCR2

#define ADMUX_CURRENT_1() { ADMUX = 0b01100000; }
#define ADMUX_CURRENT_2() { ADMUX = 0b01100010; }

// Don't configure ADCSRA outside of this. It's simply rewritten each time
// without read-modify-write.
#define START_ADC_CONV() { ADCSRA = 0b11001101; }

// Direction A (0)
// * Direction relay coil not powered
// * Motor over the bottom MOSFET
// * Non-inverted PWM output

// Direction B (1)
// * Relay coil powered
// * Motor over the top MOSFET
// * Inverted PWM output

static uint8_t mot_dirs[2];
#define MOT_1_DIR_A() { cbi(PORTE, 2); mot_dirs[0] = 0; }
#define MOT_1_DIR_B() { sbi(PORTE, 2); mot_dirs[0] = 1; }
#define MOT_2_DIR_A() { cbi(PORTB, 0); mot_dirs[1] = 0; }
#define MOT_2_DIR_B() { sbi(PORTB, 0); mot_dirs[1] = 1; }

#define MOT_1_ENABLE()	{ sbi(PORTF, 1); }
#define MOT_1_DISABLE()	{ cbi(PORTF, 1); }
#define MOT_2_ENABLE()	{ sbi(PORTF, 3); }
#define MOT_2_DISABLE()	{ cbi(PORTF, 3); }

static uint8_t pwm_requests[2];

#define MIN_PWM 0
#define MAX_PWM 250 // Bottom FET needs some on-time for the bootstrap to work.

#define PWM_INCREASE_STEP 10

volatile uint8_t overcurrent_pending[2];
volatile uint8_t latest_currents[2]; // You can read these. 1 unit = 1/2.55 A

// Convert motor currents.
ISR(ADC_vect)
{
	static uint8_t next_input = 0;
	if(next_input)
	{
		next_input = 0;
		latest_currents[1] = ADCH;
		ADMUX_CURRENT_1();
	}
	else
	{
		next_input = 1;
		latest_currents[0] = ADCH;
		ADMUX_CURRENT_2();
	}
	START_ADC_CONV();
}

// Motor 1 overcurrent
ISR(INT7_vect)
{
	if(!mot_dirs[0])
		PWM_1 = MIN_PWM;
	else
		PWM_1 = MAX_PWM;
	overcurrent_pending[0] = 2; // Setting to 2 ensures that at least one PWM cycle is really forced to min or max.
	cbi(EIMSK, 7);
}

// Motor 2 overcurrent
ISR(INT6_vect)
{
	if(!mot_dirs[1])
		PWM_2 = MIN_PWM;
	else
		PWM_2 = MAX_PWM;
	overcurrent_pending[1] = 2;
	cbi(EIMSK, 6);
}

ISR(TIMER0_OVF_vect)
{
	static uint8_t ovi_callback_cnt;

	if(overcurrent_pending[0])
	{
		overcurrent_pending[0]--;
	}
	else
	{
		sbi(EIMSK, 7); // Re-enable interrupt.
		// There has been no overcurrent event, or we have waited long enough,
		// the overcurrent condition has been cleared. -> Increase PWM towards the request.
		int16_t tmp = PWM_1;
		if(!mot_dirs[0])
		{
			tmp += PWM_INCREASE_STEP;
			if(tmp > pwm_requests[0])
				tmp = pwm_requests[0];
		}
		else
		{
			tmp -= (int16_t)PWM_INCREASE_STEP;
			if(tmp < (int16_t)MAX_PWM-(int16_t)pwm_requests[0])
				tmp = (int16_t)MAX_PWM-(int16_t)pwm_requests[0];
		}

		PWM_1 = tmp;
	}

	if(overcurrent_pending[1])
	{
		overcurrent_pending[1]--;
	}
	else
	{
		sbi(EIMSK, 6);
		int16_t tmp = PWM_2;
		if(!mot_dirs[1])
		{
			tmp += PWM_INCREASE_STEP;
			if(tmp > pwm_requests[1])
				tmp = pwm_requests[1];
		}
		else
		{
			tmp -= (int16_t)PWM_INCREASE_STEP;
			if(tmp < (int16_t)MAX_PWM-(int16_t)pwm_requests[1])
				tmp = (int16_t)MAX_PWM-(int16_t)pwm_requests[1];
		}

		PWM_2 = tmp;
	}
	if (ovi_callback_cnt++ == 255) {
		ovi_timer_callback();
	}
}

// Do not touch these, they are constrained by HW.
// Peak current limit: 2.55 units == 1A
#define MIN_SPEED 5
#define MAX_SPEED 249
#define MIN_CURR_LIMIT 1
#define MAX_CURR_LIMIT 89 // == 35A

void set_main_motor_speed(uint8_t speed)
{
	if(speed < MIN_SPEED)
		speed = MIN_SPEED;
	else if(speed > MAX_SPEED)
		speed = MAX_SPEED;

	pwm_requests[0] = speed;
}

void set_aux_motor_speed(uint8_t speed)
{
	if(speed < MIN_SPEED)
		speed = MIN_SPEED;
	else if(speed > MAX_SPEED)
		speed = MAX_SPEED;

	pwm_requests[1] = speed;
}

void set_main_motor_curr_limit(uint8_t limit)
{
	if(limit < MIN_CURR_LIMIT)
		limit = MIN_CURR_LIMIT;
	else if(limit > MAX_CURR_LIMIT)
		limit = MAX_CURR_LIMIT;

	CUR_LIMIT_PWM_1 = limit;
}

void set_aux_motor_curr_limit(uint8_t limit)
{
	if(limit < MIN_CURR_LIMIT)
		limit = MIN_CURR_LIMIT;
	else if(limit > MAX_CURR_LIMIT)
		limit = MAX_CURR_LIMIT;

	CUR_LIMIT_PWM_2 = limit;
}

void main_motor_stop() {
	set_main_motor_speed(0);
	MOT_1_DISABLE();
}

void main_motor_cw_open(uint8_t speed) {
	set_main_motor_speed(speed);
	MOT_1_DIR_A();
	MOT_1_ENABLE();
}

void main_motor_ccw_close(uint8_t speed) {
	set_main_motor_speed(speed);
	MOT_1_DIR_A();
	MOT_1_ENABLE();
}

void aux_motor_stop() {
	set_aux_motor_speed(0);
	MOT_2_DISABLE();
}

void aux_motor_cw_close(uint8_t speed) {
	set_aux_motor_speed(speed);
	MOT_2_DIR_A();
	MOT_2_ENABLE();
}

void aux_motor_ccw_open(uint8_t speed) {
	set_aux_motor_speed(speed);
	MOT_2_DIR_B();
	MOT_2_ENABLE();
}

void magnet_off() {
	PORTB &= ~(_BV(PB5));
}

void magnet_on() {
	PORTB |= _BV(PB5);

}

bool door_nearly_open() {
	return (PORTC & _BV(PC2));
}

bool door_fully_open() {
	return (PORTG & _BV(PG2));
}

bool door_fully_closed() {
	return (PORTC & _BV(PC1));
}

bool sensor_proximity() {
	return ((~PINA)&(1<<0));
}

bool button_openclose() {
	return ((~PINA)&(1<<1));
}

bool button_stop() {
	return ((~PINA)&(1<<2));
}

bool main_encoder() {
	return ((~PINC)&(1<<7));
}

bool aux_outdoor_limit() {
	return ((~PINC)&(1<<5));
}

bool aux_indoor_limit() {
	return ((~PINC)&(1<<6));
}

bool door_nearly_closed() {
	return ((~PINC)&(1<<2));
}

bool aux_encoder() {
	return ((~PINC)&(1<<3));
}

void init(void) {
	// DDR and pull-ups
				// A0 sensor_proximity (input)
				// A1 button_open (input)
				// A2 button_stop (input)
				// A3 button_close (input)
				// A4 Unused (input)
				// A5 Unused (input)
				// A6 Unused (input)
				// A7 Unused (input)


	DDRB  |= _BV(DDB0);	// B0 Aux motor dir (output)
	PORTB |= _BV(PB1);	// B1 Unused (SPI SCK) (input, pull-up)
	PORTB |= _BV(PB2);	// B2 Unused (SPI MOSI) (input, pull-up)
	PORTB |= _BV(PB3);	// B3 Unused (SPI MISO) (input, pull-up)
	DDRB  |= _BV(DDB4);	// B4 Main motor dir (output)
	DDRB  |= _BV(DDB5);	// B5 Door magnets (output)
	DDRB  |= _BV(DDB6);	// B6 Unused output (output)
	DDRB  |= _BV(DDB7);	// B7 Aux motor PWM (output)

	PORTC |= _BV(PC0);	// C0 Unused pad (input, pull-up)
				// C1 door_fully_closed (input)
				// C2 door_nearly_open (input)
				// C3 aux_encoder (input)
				// C4 door_nearly_closed (input)
				// C5 aux_indoor_limit (input)
				// C6 aux_outdoor_limit (input)
				// C7 main_encoder (input)

	DDRD |= _BV(DDD0);	// D0 Unused relay R3 (output)
	DDRD |= _BV(DDD1);	// D1 Unused relay R4 (output)
				// D2 RS-232 RXD (input)
	DDRD |= _BV(DDD3);	// D3 RS-232 TXD (output)
	DDRD |= _BV(DDD4);	// D4 RS-232 CTS (output)
	DDRD |= _BV(DDD5);	// D5 RS-232 DTR/RTS (input)
	DDRD |= _BV(DDD6);	// D6 Unused relay R2 (output)
	DDRD |= _BV(DDD7);	// D7 Door open relay (output)

	PORTE |= _BV(PE0);	// E0 Programmer (input, pull-up)
	PORTE |= _BV(PE0);	// E1 Programmer (input, pull-up)
	DDRE  |= _BV(DDE2);	// E2 Main motor dir
	DDRE  |= _BV(DDE3);	// E3 Main motor current limit (output)
	DDRE  |= _BV(DDE4);	// E4 Aux motor current limit (output)
	DDRE  |= _BV(DDE5);	// E5 Not connected (output)
				// E6 Aux motor overcurrent interrupt (input)
				// E7 Main motor overcurrent interrupt (input)

				// F0 Main motor current ADC (input)
	DDRF  |= _BV(DDF1);	// F1 Main motor enable (output)
				// F2 Aux motor current ADC (input)
	DDRF  |= _BV(DDF3);	// F3 Aux motor enable (output)
	DDRF  |= _BV(DDF4);	// F4 Not connected (output)
	DDRF  |= _BV(DDF5);	// F5 Not connected (output)
	DDRF  |= _BV(DDF6);	// F6 Not connected (output)
	DDRF  |= _BV(DDF7);	// F7 Not connected (output)

	DDRG  |= _BV(PG0);	// G0 Unused relay R5 (output)
	DDRG  |= _BV(PG1);	// G1 Unused relay R6 (output)
				// G3 door_fully_open (input)
	DDRG  |= _BV(PG4);	// G4 Unused output FETOUT4 (output)

	// UART

	UBRR1H = UBRRH_VALUE;		// Baud rate
	UBRR1L = UBRRL_VALUE;

#if USE_2X
	UCSR1A |= _BV(U2X1);		// UART double speed
#else
	UCSR1A &= ~(_BV(U2X1));
#endif

	UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10); // 8-bit data
	UCSR1B |= _BV(RXEN1) | _BV(TXEN1);   // Enable RX and TX

	//	Motor control

	// Timer3 Channels A and B generate PWM signals for RC DACs to generate
	// reference levels for analog current limits.
	//
	//	* 8-bit "Fast PWM" mode is used (mode = 5)
	//	* Non-inverting PWM generation (COM bits 0b10)
	//	* Prescaler = 1 for maximum frequency. (27 kHz at 7 MHz)

	TCCR3A = 0b10100001;
	TCCR3B = 0b00001001;
	CUR_LIMIT_PWM_1 = 10;
	CUR_LIMIT_PWM_2 = 10;
	pwm_requests[0] = 0;
	pwm_requests[1] = 0;

	// Timer0 and Timer2 provide 8-bit PWM signals to drive the motor half
	// bridge.
	//	* 8-bit "Fast PWM" mode is used (mode = 3)
	//	* Non-inverting PWM generation (COM bits 0b10)
	//	* Prescaler = 8, PWM freq @ 7 MHz = 3.5 kHz

	TCCR0 = 0b01101010;
	PWM_1 = 10;
	TCCR2 = 0b01101010;
	PWM_2 = 10;
	DDRB |= 1<<4 | 1<<7;
	TIMSK |= 1<<0; // Periodic interrupt (Timer0 overflow) to process PWM settings.

	// External interrupts
	//	* INT7 = Motor 1 overcurrent
	//	* INT6 = Motor 2 overcurrent
	//	* Rising edge sensitive

	EICRB = 0b11110000;
	EIMSK |= 1<<7 | 1<<6;

	// ADC:
	//	* Used in single-shot mode
	//	* ISR changes the channel and starts the next conversion
	//	* With XTAL=7327000 and prescaler=32, fADC=228968kHz
	//	* Result is left-adjusted, only ADCH is read (8-bit data).

	ADMUX_CURRENT_1();
	START_ADC_CONV();


	// MOTOR CONTROLLER INIT END

	sei();
}


