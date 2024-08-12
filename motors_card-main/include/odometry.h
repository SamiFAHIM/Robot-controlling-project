
#ifndef ODOM_HPP
#define ODOM_HPP

#include "structs.h"
#include "Arduino.h"

Odom get_position();
void set_position(Odom new_odom);
void update_odometer(uint32_t dt);

double get_left_wheel_speed();
double get_right_wheel_speed();

#endif