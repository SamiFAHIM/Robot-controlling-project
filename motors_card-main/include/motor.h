#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "pins.h"



void set_speed_motor_right(int dir, int speed);
void enable_pwm_motor_right(int enabled);
void init_motor_right(int decay_mode, int current_limit);


void set_speed_motor_left(int dir, int speed);
void enable_pwm_motor_left(int enabled);
void init_motor_left(int decay_mode, int current_limit);


#define MODE_SLOW_DECAY 1
#define MODE_FAST_DECAY 0

#define DIR_STOP 0
#define DIR_BACKWARD -1
#define DIR_FORWARD 1
#define DIR_SIGNED 2

#endif