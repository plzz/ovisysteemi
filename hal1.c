#define BAUD 57600

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/setbaud.h>

#include "hal1.h"

int const MAIN_MOTOR_PWM_TOP = 255;
int const MAIN_MOTOR_BRAKE_SPEED = 120;
int const MAIN_MOTOR_MIN_SPEED = 150;
int const MAIN_MOTOR_MED_SPEED = 254;
int const MAIN_MOTOR_MAX_SPEED = 255;
int const MAGNET_OPEN_WAIT = 5;          // 10ths of a second

// Hardware abstraction layer

// Globals

// Requested speed, used by ISR to accelerate
static volatile uint8_t main_motor_speed_request;
static volatile uint8_t main_motor_speed_unscaled;

void init(void) {
	// DDR and pull-ups

					// B0 door_nearly_open (input)
					// B1 door_fully_closed (input)
	DDRB |= _BV(DDB2);		// B2 OC1B = main motor pwm (output)
					// B3 sensor_proximity (input) (parallel with phys. open button)
					// B4 button_openclose (input) (physically labeled: close)
					// B5 button_stop (input)
					// B6 XTAL1, unused (input)
					// B7 XTAL2, unused (input)

	PORTB |= _BV(PB3)		// B3 sensor_proximity
	       | _BV(PB4)		// B4 BUTTON_CLOSE internal pull-up
	       | _BV(PB5);		// B5 BUTTON_STOP internal pull-up

	DDRC |= _BV(DDC0)		// C0 Main motor direction (output)
	      | _BV(DDC1)		// C1 Main motor enable (output)
	      | _BV(DDC2)		// C2 Auxiliary motor enable (output)
	      | _BV(DDC3)		// C3 Auxiliary motor direction (output)
	      | _BV(DDC4)		// C4 Magnet enable (output)
	      | _BV(DDC5);		// C5 Main PWM bypass (output)
					// C6 reset, unused (input)

	// Initialize as high (off)
	PORTC |= _BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5);

					// D0 RS232 RX (input)
	DDRD |= _BV(DDD1);		// D1 RS232 TX (output)
					// D2 door_fully_open (input)
					// D3 main_encoder (input)
					// D4 aux_outdoor_limit (input)
					// D5 aux_indoor_limit (input)
					// D6 door_nearly_closed (input)
					// D7 aux_encoder (input)

	// UART

	UBRR0H = UBRRH_VALUE;		// Baud rate
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);		// UART double speed
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data
	UCSR0B |= _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX

	// Timer

	OCR1B = 0;			// Initial motor speed 0 = 0%

	TCCR1A |= _BV(COM1B1)		// Set OC1B on Compare Match
		| _BV(WGM10);		// Fast PWM

	TCCR1B |= _BV(CS11)		// CLKio/8
		| _BV(WGM12);


	TIMSK0 |=_BV(TOIE0);		// Enable timer 0 overflow interrupt
	OCR0A = 99;			// Timer 0 compare value
	TCCR0B = _BV(CS02) | _BV(CS00);	// Divide timer 0 clock by 1024
	sei();				// Enable interrupts
}

ISR(TIMER0_OVF_vect) {
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;
	if (main_motor_speed_request > main_motor_speed_unscaled)
		main_motor_speed_unscaled++;

	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;
	if (main_motor_speed_request < main_motor_speed_unscaled)
		main_motor_speed_unscaled--;

	if (main_motor_speed_unscaled && main_motor_speed_unscaled < 16)
		set_main_motor_speed(16);
	else
		set_main_motor_speed(main_motor_speed_unscaled);

	if (main_motor_speed_unscaled == 0) {
		OCR1B = 0;			// Zero speed
		PORTC |= _BV(PC0)		// Set direction
		       | _BV(PC1);		// Set enable
	}
}


void main_motor_stop() {
	main_motor_speed_request = 0;
}

void main_motor_cw_open(uint8_t speed) {
	main_motor_speed_request = speed;
	PORTC |= _BV(PC0);		// Set direction -> CW (open)
	PORTC &= ~(_BV(PC1));		// Clear enable
}

void main_motor_ccw_close(uint8_t speed) {
	main_motor_speed_request = speed;
	PORTC &= ~(_BV(PC0)		// Clear direction -> CCW (close)
	         | _BV(PC1));		// Clear enable
}

static void set_main_motor_speed(int speed) {
	OCR1B = speed;
}

int get_main_motor_speed() {
	return OCR1B;
}

void aux_motor_stop() {
	PORTC |= _BV(PC3)		// Set direction
	       | _BV(PC2);		// Set enable
}

void aux_motor_cw_close(uint8_t speed) {
	PORTC |= _BV(PC3);		// Set direction -> CW (open)
	PORTC &= ~(_BV(PC2));		// Clear enable
}

void aux_motor_ccw_open(uint8_t speed) {
	PORTC &= ~(_BV(PC3)		// Set direction -> CCW (close)
	         | _BV(PC2));		// Set enable
}

void magnet_off() {
	PORTC &= ~(_BV(PC4));
}

void magnet_on() {
	PORTC |= _BV(PC4);
}

void main_motor_bypass_on() {
	PORTC &= ~(_BV(PC5));
}

void main_motor_bypass_off() {
	PORTC |= _BV(PC5);
}

bool door_nearly_open() {
	return PINB & _BV(PB0);
}

bool door_fully_closed() {
	return PINB & _BV(PB1);
}

bool sensor_proximity() {
	return !(PINB & _BV(PB3));
}

bool button_openclose() {
	return !(PINB & _BV(PB4));
}

bool button_stop() {
	return PINB & _BV(PB5);
}

bool door_fully_open() {
	return PIND & _BV(PD2);
}

bool main_encoder() {
	return PIND & _BV(PD3);
}

bool aux_outdoor_limit() {
	return PIND & _BV(PD4);
}

bool aux_indoor_limit() {
	return PIND & _BV(PD5);
}

bool door_nearly_closed() {
	return PIND & _BV(PD6);
}

bool aux_encoder() {
	return PIND & _BV(PD7);
}


