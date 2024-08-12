#ifndef COMM_HPP
#define COMM_HPP


#include <Arduino.h>
#include "structs.h"

void init_communication();
void ros_spin_once_communication();

void publish_status(int32_t status);
void publish_odom(Odom odom);

void check_and_reset_for_ros_agent_connected();

#endif