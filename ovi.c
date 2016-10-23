#include <avr/io.h>
#include <stdbool.h>

// Configurable constants

int const MAIN_MOTOR_PWM_TOP = 255;
int const MAIN_MOTOR_MIN_SPEED = 30;
int const MAGNET_OPEN_WAIT = 5;		// 10ths of a second

// Hardware abstraction layer

void init(void) {
					// B0 SENSOR_AUX_STANDBY (input)
					// B1 SENSOR_AUX_BACK (input)
	DDRB |= _BV(DDB2)		// B2 OC1B = main motor pwm (output)
	      | _BV(DDB3)		// B3 unused (output)
	      | _BV(DDB4)		// B4 unused (output)
	      | _BV(DDB5);		// B5 unused (output)

	DDRC |= _BV(DDC0)		// C0 Main motor direction (output)
	      | _BV(DDC1)		// C1 Main motor enable (output)
	      | _BV(DDC2)		// C2 Auxiliary motor enable (output)
	      | _BV(DDC3)		// C3 Auxiliary motor direction (output)
	      | _BV(DDC4)		// C4 Magnet enable (output)
	      | _BV(DDC5)		// C5 unused relay
	      | _BV(DDC6);		// C6 unused

					// D0 RS232 RX (input)
	DDRD |= _BV(DDD1);		// D1 RS232 TX (output)
					// D3 SENSOR_AUX_MOTOR_PULSE (input)
					// D4 SENSOR_DOOR_CLOSED (input)
					// D5 SENSOR_DOOR_OPEN (input)
					// D6 SENSOR_DOOR_NEARLY_OPEN (input)
					// D7 SENSOR_AUX_FRONT (input)

	TCCR1A |= _BV(COM1B1)		// Clear OC1B on Compare Match
		| _BV(WGM11)
		| _BV(WGM12);		// Fast PWM

	TCCR1B |= _BV(CS12)		// CLKio/256
		| _BV(WGM13);		// TOP = ICR1

	ICR1 = MAIN_MOTOR_PWM_TOP;
	OCR1B = 0;			// Initial motor speed 0 = 0%
}

void main_motor_stop() {
	OCR1B = 0;			// Zero speed
	PORTC &= ~(_BV(PC0)		// Clear direction
		 | _BV(PC1));		// Clear enable
}

void main_motor_cw_open() {
	PORTC &= ~(_BV(PC0));		// Clear direction -> CW (open)
	PORTC |= _BV(PC1);		// Set enable
}

void main_motor_ccw_close() {
	PORTC |= _BV(PC0)		// Set direction -> CCW (close)
	       | _BV(PC1);		// Set enable
}

void set_main_motor_speed(int speed) {
	OCR1B = speed;
}

int get_main_motor_speed() {
	return OCR1B;
}

void aux_motor_stop() {
	PORTC &= ~(_BV(PC3)		// Clear direction
		 | _BV(PC2));		// Clear enable
}

void aux_motor_cw_close() {
	PORTC &= ~(_BV(PC3));		// Clear direction -> CW (open)
	PORTC |= _BV(PC2);		// Set enable
}

void aux_motor_ccw_open() {
	PORTC |= _BV(PC3)		// Set direction -> CCW (close)
	       | _BV(PC2);		// Set enable
}

void magnet_on() {
	PORTC |= _BV(PC4);
}

void magnet_off() {
	PORTC &= ~(_BV(PC4));
}

bool sensor_main_encoder() {
	return PORTD | _BV(PD2);
}

bool aux_encoder() {
	return PORTD | _BV(PD3);
}

bool door_fully_closed() {
	return PORTD | _BV(PD4);
}

bool door_fully_open() {
	return PORTD | _BV(PD5);
}

bool door_nearly_open() {
	return PORTD | _BV(PD6);
}

bool aux_front() {
	return PORTD | _BV(PD7);
}

bool aux_standby() {
	return PORTB | _BV(PB0);
}

bool aux_back() {
	return PORTB | _BV(PB1);
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
	       S_CLOSING3 };

enum err_t { E_NOERR = 0,
             E_TIMEOUT = 1,
             E_MAIN_MOTOR_FAULT = 2,
             E_AUX_MOTOR_FAULT = 3 };

// These counters are all incremented by a timer interrupt every 100ms.
uint8_t		s_opening1_counter;		// Delay counter for S_OPENING1
uint16_t	movement_timeout_counter;	// Movement timeout
uint8_t		main_motor_encoder_counter;
uint8_t		aux_motor_encoder_counter;

int main (void)
{
	init();
	
	enum state_t state = S_STOP;
	enum err_t err = E_NOERR;

	while(1) switch (state) {
		// Initial state if door is not conclusively open or closed.
		// Also entered in case of an error.
		case S_STOP:
			main_motor_stop();
			aux_motor_stop();
			break;

		// Door is fully open. Motors stopped, magnet off.
		case S_OPEN:
			main_motor_stop();
			aux_motor_stop();
			magnet_off();
			break;

		// Door is fully open. Motors stopped, magnet on.
		case S_CLOSED:
			main_motor_stop();
			aux_motor_stop();
			magnet_on();
			break;

		// Open magnets, clear timer
		case S_OPENING1:
			magnet_off();
			s_opening1_counter = 0;
			state = S_OPENING2;
			break;

		// Wait
		case S_OPENING2:
			if (s_opening1_counter = MAGNET_OPEN_WAIT) {
				state = S_OPENING3;
			}
			break;

		// Start the auxiliary motor to open the middle of the door.
		// Wait until SENSOR_DOOR_CLOSED opens.
		case S_OPENING3:
			aux_motor_ccw_open();
			if (sensor_door_closed() == 0) {
				state = S_OPENING4;
			}
			break;

		// Start accelerating the main motor.
		case S_OPENING4:
			main_motor_speed(MAIN_MOTOR_MIN_SPEED);
			main_motor_cw_open();
			main_motor_accelerate();
			state = S_OPENING5;
			break;

 		// The door is open enough to enable the main motor.
		// Run the aux motor until SENSOR_AUX_STANDBY is reached.
		// Run the main motor until SENSOR_DOOR_NEARLY_OPEN is reached.
		case S_OPENING5:
			if (sensor_aux_standby()) {
				aux_motor_stop();
			}
			if (sensor_door_nearly_open()) {
				main_motor_decelerate();
			}
			if (sensor_door_open()) {
				main_motor_stop();
				aux_motor_stop();
				state = S_OPEN;
			}
			break;

		case S_CLOSING1:
			break;

		case S_CLOSING2:
			break;

		case S_CLOSING3:
			break;

		default:
			if (main_motor_fault()) {
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
			break;
	}
}

