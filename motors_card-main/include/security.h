
#ifndef SEC_HPP
#define SEC_HPP

#define STATUS_OFF_OK 0
#define STATUS_ON_OK 1
#define STATUS_OVERCURRENT_LEFT 10
#define STATUS_OVERCURRENT_RIGHT 11
#define STATUS_OVERCURRENT_BOTH 12
#define STATUS_CMDVEL_TIMEOUT 3

#include "Arduino.h"

uint32_t get_status();
uint32_t update_security(uint32_t dt);

#endif