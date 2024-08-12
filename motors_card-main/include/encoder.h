#ifndef ENCODER_HPP
#define ENCODER_HPP


#include <Arduino.h>
#include "hardware/gpio.h"
#include "pins.h"


void init_encoders();
void update_encoders_ticks();


int32_t get_ticks_encoder_left();
int32_t get_ticks_encoder_right();

#endif