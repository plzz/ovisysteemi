#ifndef HAL2_H
#define HAL2_H

// Mechanical constants

extern int const MAIN_MOTOR_PWM_TOP;
extern int const MAIN_MOTOR_BRAKE_SPEED;
extern int const MAIN_MOTOR_MIN_SPEED;
extern int const MAIN_MOTOR_MED_SPEED;
extern int const MAIN_MOTOR_MAX_SPEED;
extern int const MAGNET_OPEN_WAIT;

void init(void);

void main_motor_stop();
void main_motor_cw_open(uint8_t speed);
void main_motor_ccw_close(uint8_t speed);
void aux_motor_stop();
void aux_motor_cw_close();
void aux_motor_ccw_open();
void magnet_off();
void magnet_on();
bool door_nearly_open();
bool door_fully_open();
bool door_fully_closed();

bool sensor_proximity();
bool button_openclose();
bool button_stop();
bool main_encoder();
bool aux_outdoor_limit();
bool aux_indoor_limit();
bool door_nearly_closed();
bool aux_encoder();

#endif
