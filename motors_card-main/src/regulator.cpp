#include "regulator.h"
#include "motor.h"
#include "odometry.h"
#include "params.h"
#include "PID.hpp"


PID left_motor_pid(-500, 500);
PID right_motor_pid(-500, 500);

double current_target_lin = 0;
double current_target_ang = 0;

void init_regulator(){
    left_motor_pid.setGains(get_left_motor_pid_params());
    right_motor_pid.setGains(get_right_motor_pid_params());
}

void update_regulator(uint32_t dt){


    left_motor_pid.setSetpoint(current_target_lin - current_target_ang * (get_geometry().distance_between_encoders / 2.0));
    right_motor_pid.setSetpoint(current_target_lin + current_target_ang * (get_geometry().distance_between_encoders / 2.0));
// il faudra get les linear speed et les angular speed des robots
    left_motor_pid.setProcessValue(get_left_wheel_speed());
    right_motor_pid.setProcessValue(get_right_wheel_speed());


// il y aura un compute angular speed et un compute linear speed
    int current_cmd_motor_left = left_motor_pid.compute(dt);
    double current_cmd_motor_right = right_motor_pid.compute(dt);

    // il faudra sommer une commande linéaire et une commande angulaire
    set_speed_motor_left(DIR_SIGNED, -current_cmd_motor_left); // ne cherchez pas, c'est normal 
    set_speed_motor_right(DIR_SIGNED, current_cmd_motor_right);
}



void set_speed_target(double lin, double ang){
    current_target_lin = lin;
    current_target_ang = ang;
}
