#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>

#include "ovi.h"
#include "hal2.h"

// Globals

// These counters are all incremented by a timer interrupt every 100ms.
volatile uint8_t s_opening1_timer;		// Delay counter for S_OPENING1
volatile uint8_t s_opening4_timer;
volatile uint8_t s_closing5_timer;
volatile uint8_t s_closing8_timer;

volatile uint8_t status_dump_timer;

volatile uint16_t movement_timeout_counter;	// Movement timeout
volatile uint8_t main_motor_encoder_counter;
volatile uint8_t aux_motor_encoder_counter;

// Main state machine

enum state_t { S_STOP,
	       S_OPEN,
	       S_CLOSED,

	       S_OPENING1,
	       S_OPENING2,
	       S_OPENING3,
	       S_OPENING4,
	       S_OPENING5,
	       S_OPENING6,

	       S_CLOSING1,
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

// Callback for timer interrupt. Increments counters.
void ovi_timer_callback(void) {
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

	if (++status_dump_timer  > 252) {
                status_dump_timer = 252;
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

int uart_getchar_nonblock() {
	if (UCSR0A & _BV(RXC0)) {
		return UDR0;
	} else {
		return EOF;
	}
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void handle_io(enum state_t *state) {
	if (status_dump_timer == 252) {
		status_dump_timer = 0;
		printf ("%%STATE:");
		if (door_fully_open())		printf(" door_fully_open");
		if (door_fully_closed())	printf(" door_fully_closed");
		if (sensor_proximity())		printf(" sensor_proximity");
		if (button_openclose())		printf(" button_openclose");
		if (button_stop())		printf(" button_stop");
		if (aux_outdoor_limit())	printf(" aux_outdoor_limit");
		if (aux_indoor_limit())		printf(" aux_indoor_limit");
		if (door_nearly_closed())	printf(" door_nearly_closed");

		printf( " state=%d", *state);
		printf("\r\n");
	}

	switch (uart_getchar_nonblock()) {
	case 'o':
		if ((*state == S_CLOSED) || (*state == S_STOP))
			if (door_fully_closed()) *state = S_OPENING1;
		break;

	case 'c':
		if ((*state == S_OPEN) || (*state == S_STOP))
			if (door_fully_open()) *state = S_CLOSING1;
		break;

	case 's':
		*state = S_STOP;
		break;

	case 'z': // aux open
		aux_motor_ccw_open(255);
		_delay_ms(250);
		aux_motor_stop();
		break;

	case 'x': // aux close
		aux_motor_cw_close(255);
		_delay_ms(250);
		aux_motor_stop();
		break;

	case 'v': // main open
		main_motor_cw_open(254);
		_delay_ms(250);
		main_motor_stop();
		break;

	case 'b': // main close
		main_motor_ccw_close(254);
		_delay_ms(250);
		main_motor_stop();
		break;


	}
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
			// Wait until door_nearly_closed opens.
			case S_OPENING3:
				aux_motor_ccw_open(255);
				main_motor_cw_open(MAIN_MOTOR_MED_SPEED);
				if (door_nearly_closed()) {
					state = S_OPENING4;
					s_opening4_timer = 0;
				}
				break;
	
			// Unused
			case S_OPENING4:
				if (s_opening4_timer > 50) {
					state = S_OPENING5;
				} else {
					aux_motor_stop();
					main_motor_cw_open(MAIN_MOTOR_MED_SPEED);
				}
				break;
	
			// The door is open enough to enable the main motor.
			// Run the aux motor until aux_indoor_limit is reached.
			// Run the main motor at full speed until door_nearly_open is reached.
			case S_OPENING5:
				if (aux_indoor_limit()) {
					aux_motor_stop();
				} else {
					aux_motor_ccw_open(255);
				}
				if (door_nearly_open()) {
					main_motor_cw_open(MAIN_MOTOR_BRAKE_SPEED);
					state = S_OPENING6;
				} else if (door_fully_open()) {
					main_motor_stop();
					state = S_OPENING6;
				} else {
					if (aux_indoor_limit()) {
						main_motor_cw_open(MAIN_MOTOR_MAX_SPEED);
					} else {
						main_motor_cw_open(MAIN_MOTOR_MED_SPEED);
					}
				}
				break;

			case S_OPENING6:
				if (aux_indoor_limit()) {
					aux_motor_stop();
				} else {
					aux_motor_ccw_open(255);
				}
				if (door_fully_open()) {
					main_motor_stop();
				} else {
					main_motor_cw_open(MAIN_MOTOR_BRAKE_SPEED);
				}

				if (aux_indoor_limit() && door_fully_open())
					state = S_OPEN;

				break;

			case S_CLOSING1:
				if (aux_indoor_limit()) {
					aux_motor_stop();
					state = S_CLOSING3;
				} else {
					aux_motor_cw_close(255);
				}
				break;

			// Run at max speed until door_nearly_open is reached
			case S_CLOSING3:
				if (sensor_proximity()) {
					_delay_ms(1);
		                        if (sensor_proximity()) main_motor_stop();
					_delay_ms(1000);
				} else if (!door_nearly_open()) {
					main_motor_ccw_close(MAIN_MOTOR_MED_SPEED);
				} else {
					main_motor_ccw_close(MAIN_MOTOR_MAX_SPEED);
					state = S_CLOSING4;
				}

				// Skip to S_CLOSING5 if we somehow already reach door_nearly_closed
				if (door_nearly_closed()) {
                                        s_closing5_timer = 0;
                                        state = S_CLOSING5;
                                }
				// Skip to S_CLOSING9 if we somehow already reached door_fully_closed
				if (door_fully_closed()) {
                                        state = S_CLOSING9;
                                }
				break;

			// Wait until door_nearly_closed is reached.
			case S_CLOSING4:
				if (sensor_proximity()) {
					_delay_ms(1);
					if (sensor_proximity()) main_motor_stop();
					_delay_ms(1000);
				} else if (door_nearly_closed()) {
					s_closing5_timer = 0;
					state = S_CLOSING5;
				} else {
					main_motor_ccw_close(MAIN_MOTOR_MAX_SPEED);
				}
				break;

			// Wait until timeout, then slow down main motor to
			// minimum speed and start auxiliary motor.
			case S_CLOSING5:
				if (s_closing5_timer > 15) {
					state = S_CLOSING7;
				} else {
					main_motor_ccw_close(MAIN_MOTOR_MAX_SPEED);
				}
				break;
	
			case S_CLOSING7:
				if (sensor_proximity()) {
					_delay_ms(1);
					if (sensor_proximity()) {
						main_motor_stop();
						aux_motor_stop();
					}
					_delay_ms(1000);
				} else if (door_fully_closed()) {
					magnet_on();
					s_closing8_timer = 0;
					state = S_CLOSING8;
				} else {
					aux_motor_cw_close(255);
					main_motor_ccw_close(MAIN_MOTOR_MIN_SPEED);
				}
				break;
	
			case S_CLOSING8:
				if (s_closing8_timer > 75) {
					main_motor_stop();
					state = S_CLOSING9;
				}
				break;
	
			case S_CLOSING9:
				if (aux_outdoor_limit()) {
					aux_motor_stop();
					state = S_CLOSED;
				} else {
					aux_motor_ccw_open(255);
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
		} */
	
		handle_io(&state);
		if (button_stop()) { _delay_ms(20); if (button_stop()) state = S_STOP; }

	}
}

