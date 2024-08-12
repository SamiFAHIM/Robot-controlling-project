
#include "params.h"
#include "Arduino.h"

Geometry robot_geometry;

PIDparams left_motor_pid_params;
PIDparams right_motor_pid_params;

PIDparams get_left_motor_pid_params(){
    return left_motor_pid_params;
}

PIDparams get_right_motor_pid_params(){
    return right_motor_pid_params;
}

Geometry get_geometry(){
    return robot_geometry;
}

void set_geometry(Geometry geometry){
    robot_geometry = geometry;
}

void set_left_motor_pid_params(PIDparams params){
    left_motor_pid_params = params;
}

void set_right_motor_pid_params(PIDparams params){
    right_motor_pid_params = params;
}

void init_all_params(){
    robot_geometry.distance_between_encoders = 0.35;
    robot_geometry.left_encoder_conversion_factor = TWO_PI*0.031/(1000*4);
    robot_geometry.right_encoder_conversion_factor = TWO_PI*0.031/(1000*4);


    left_motor_pid_params.Kd = 0;
    left_motor_pid_params.Ki = 8.0;
    left_motor_pid_params.Kp = 300.0;

    right_motor_pid_params.Kd = 0;
    right_motor_pid_params.Ki = 8.0;
    right_motor_pid_params.Kp = 200.0;
}