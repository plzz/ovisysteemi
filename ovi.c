#define BAUD 9600

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/setbaud.h>

// Configurable constants

int const MAIN_MOTOR_PWM_TOP = 255;
int const MAIN_MOTOR_BRAKE_SPEED = 120;
int const MAIN_MOTOR_MIN_SPEED = 150;
int const MAIN_MOTOR_MED_SPEED = 254;
int const MAIN_MOTOR_MAX_SPEED = 255;
int const MAGNET_OPEN_WAIT = 5;		// 10ths of a second

// Globals

// These counters are all incremented by a timer interrupt every 100ms.
volatile uint8_t s_opening1_timer;		// Delay counter for S_OPENING1
volatile uint8_t s_opening4_timer;
volatile uint8_t s_closing5_timer;
volatile uint8_t s_closing8_timer;

volatile uint16_t movement_timeout_counter;	// Movement timeout
volatile uint8_t main_motor_encoder_counter;
volatile uint8_t aux_motor_encoder_counter;

// Requested speed, used by ISR to accelerate
volatile uint8_t main_motor_speed_request;
volatile uint8_t main_motor_speed_unscaled;

// Hardware abstraction layer

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

	TCCR1A |= _BV(COM1B0)
		| _BV(COM1B1)		// Set OC1B on Compare Match
		| _BV(WGM11)
		| _BV(WGM12);		// Fast PWM

	TCCR1B |= _BV(CS12)		// CLKio/256
		| _BV(WGM13);		// TOP = ICR1

	ICR1 = MAIN_MOTOR_PWM_TOP;
	OCR1B = 0;			// Initial motor speed 0 = 0%

	TIMSK0 |=_BV(TOIE0);		// Enable timer 0 overflow interrupt
	OCR0A = 99;			// Timer 0 compare value
	TCCR0B = _BV(CS02) | _BV(CS00);	// Divide timer 0 clock by 1024
	sei();				// Enable interrupts
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

void set_main_motor_speed(int speed) {
	OCR1B = speed;
}

int get_main_motor_speed() {
	return OCR1B;
}

void aux_motor_stop() {
	PORTC |= _BV(PC3)		// Set direction
	       | _BV(PC2);		// Set enable
}

void aux_motor_cw_close() {
	PORTC |= _BV(PC3);		// Set direction -> CW (open)
	PORTC &= ~(_BV(PC2));		// Clear enable
}

void aux_motor_ccw_open() {
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
	return PINB & _BV(PB3);
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

// Main state machine

enum state_t { S_STOP,
	       S_OPEN,
	       S_CLOSED,

	       S_OPENING1,
	       S_OPENING2,
	       S_OPENING3,
	       S_OPENING4,
	       S_OPENING5,

	       S_CLOSING1,
	       S_CLOSING2,
	       S_CLOSING3,
	       S_CLOSING4,
	       S_CLOSING5,
	       S_CLOSING7,
	       S_CLOSING8,
	       S_CLOSING9 };

enum err_t { E_NOERR = 0,
             E_TIMEOUT = 1,
             E_MAIN_MOTOR_FAULT = 2,
             E_AUX_MOTOR_FAULT = 3 };

ISR(TIMER0_OVF_vect) {
	TCNT0 = 0; // Zero the counter again.

	if (++s_opening1_timer > 252) {
		s_opening1_timer = 252;
	}

	if (++s_opening4_timer > 252) {
		s_opening4_timer = 252;
	}

	if (++s_closing5_timer > 252) {
		s_closing5_timer = 252;
	}

	if (++s_closing8_timer > 252) {
		s_closing8_timer = 252;
	}

	if (++movement_timeout_counter == 0) {
		movement_timeout_counter = 65535;
	}

	if (++main_motor_encoder_counter == 0) {
		main_motor_encoder_counter = 255;
	}

	if (++aux_motor_encoder_counter == 0) {
		aux_motor_encoder_counter = 255;
	}

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

	if (main_motor_speed_unscaled == 255)
		main_motor_bypass_on();
	else
		main_motor_bypass_off();

	if (main_motor_speed_unscaled == 0) {
		OCR1B = 0;			// Zero speed
		PORTC |= _BV(PC0)		// Set direction
		       | _BV(PC1);		// Set enable
	}

}

int uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);	// Wait until UART is free.
	UDR0 = c;
	return 0;
}

int uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0);	// Wait until data exists.
	return UDR0;
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void handle_io(enum state_t *state) {
}

int main (void)
{
	init();

	stdout = stdin = &uart_io;

	enum state_t state = S_STOP;
	enum err_t err = E_NOERR;

	_delay_ms(1000);

	if (door_fully_closed()) state = S_CLOSED;
	else if (door_fully_open()) state = S_OPEN;

	while(1) {
		switch (state) {
			// Initial state if door is not conclusively open or closed.
			// Also entered in case of an error.
			case S_STOP:
				main_motor_stop();
				aux_motor_stop();
				if (button_openclose()) {
					_delay_ms(20);
					if (button_openclose()) {
						if (door_fully_open()) state = S_CLOSING1;
						if (door_fully_closed()) state = S_OPENING1;
					}
				}
				break;
	
			// Door is fully open. Motors stopped, magnet off.
			case S_OPEN:
				if (button_openclose()) {
					_delay_ms(20);
					if (button_openclose()) state = S_CLOSING1;
				}
				main_motor_stop();
				aux_motor_stop();
				magnet_off();
				break;
	
			// Door is fully closed. Motors stopped, magnet on.
			case S_CLOSED:
				if (button_openclose()) {
					_delay_ms(20);
					if (button_openclose()) state = S_OPENING1;
				}
				main_motor_stop();
				aux_motor_stop();
				magnet_on();
				break;
	
			// Open magnets, clear timer
			case S_OPENING1:
				magnet_off();
				s_opening1_timer = 0;
				state = S_OPENING2;
				break;
	
			// Wait
			case S_OPENING2:
				if (s_opening1_timer > MAGNET_OPEN_WAIT) {
					state = S_OPENING3;
				}
				break;
	
			// Start the auxiliary motor to open the middle of the door.
			// Wait until door_fully_closed opens.
			case S_OPENING3:
				aux_motor_ccw_open();
				main_motor_cw_open(MAIN_MOTOR_MED_SPEED);
				if (door_nearly_closed()) {
					state = S_OPENING4;
					s_opening4_timer = 0;
				}
				break;
	
			// Start accelerating the main motor.
			case S_OPENING4:
				if (s_opening4_timer > 30) {
					main_motor_cw_open(MAIN_MOTOR_MAX_SPEED);
					state = S_OPENING5;
				}
				break;
	
			// The door is open enough to enable the main motor.
			// Run the aux motor until aux_indoor_limit is reached.
			// Run the main motor at full speed until door_nearly_open is reached.
			case S_OPENING5:
				if (aux_indoor_limit()) {
					aux_motor_stop();
					if (door_fully_open()) state = S_OPEN;
				}
				if (door_nearly_open()) {
					main_motor_cw_open(MAIN_MOTOR_BRAKE_SPEED);
				}
				if (door_fully_open()) {
					main_motor_stop();
				}
				break;

			case S_CLOSING1:
				if (aux_indoor_limit()) {
					aux_motor_stop();
					state = S_CLOSING2;
				} else {
					aux_motor_cw_close();
				}
				break;

			// Start main motor
			case S_CLOSING2:
				main_motor_ccw_close(MAIN_MOTOR_MED_SPEED);
				state = S_CLOSING3;
				break;
	
			// Run until door_nearly_open is reached, then
			// accelerate
			case S_CLOSING3:
				if (door_nearly_open() == 1) {
					main_motor_ccw_close(MAIN_MOTOR_MAX_SPEED);
					state = S_CLOSING4;
				}
				break;

			// Wait until door_nearly_closed is reached.
			case S_CLOSING4:
				if (door_nearly_closed()) {
					s_closing5_timer = 0;
					state = S_CLOSING5;
				}
				break;

			// Wait until timeout, then slow down main motor to
			// minimum speed and start auxiliary motor.
			case S_CLOSING5:
				if (s_closing5_timer > 15) {
					aux_motor_cw_close();
					main_motor_ccw_close(MAIN_MOTOR_MIN_SPEED);
					state = S_CLOSING7;
				}
				break;
	
			case S_CLOSING7:
				if (door_fully_closed()) {
					magnet_on();
					s_closing8_timer = 0;
					state = S_CLOSING8;
				}
				break;
	
			case S_CLOSING8:
				if (s_closing8_timer > 75) {
					main_motor_stop();
					aux_motor_ccw_open();
					state = S_CLOSING9;
				}
				break;
	
			case S_CLOSING9:
				if (aux_outdoor_limit()) {
					aux_motor_stop();
					state = S_CLOSED;
				}
				break;
	
		}
		/* if (main_motor_fault()) {
			err = E_MAIN_MOTOR_FAULT;
			state = S_STOP;
		}
		if (aux_motor_fault()) {
			err = E_AUX_MOTOR_FAULT;
			state = S_STOP;
		}
		if (timeout()) {
			err = E_TIMEOUT;
			state = S_STOP;
		}
	
		handle_io(&state); */
		if (button_stop()) { _delay_ms(20); if (button_stop()) state = S_STOP; }

		if (sensor_proximity()) {
			_delay_ms(20);
			if (sensor_proximity()) state = S_STOP;
		}
	}
}

