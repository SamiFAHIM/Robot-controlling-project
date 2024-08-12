
#ifndef REG_HPP
#define REG_HPP

#include "Arduino.h"

void update_regulator(uint32_t dt);
void set_speed_target(double lin, double ang);
void init_regulator();




#endif