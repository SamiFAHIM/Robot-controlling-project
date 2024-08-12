#ifndef CLA_HPP
#define CLA_HPP



#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "pins.h"


void init_clamp_left();
void set_clamp_left_angle_and_enable(int enable, int angle);

void init_clamp_right();
void set_clamp_right_angle_and_enable(int enable, int angle);
uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d);


void open_clamps();
void close_clamps();
void activate_dance_mode();
void servos_dance_update();

#define RIGHT_OPEN_ANGLE 305
#define RIGHT_CLOSE_ANGLE 293

#define LEFT_OPEN_ANGLE 280
#define LEFT_CLOSE_ANGLE 292

#endif