#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

typedef struct Geometry {
   double distance_between_encoders;
   double right_encoder_conversion_factor;
   double left_encoder_conversion_factor;
} Geometry;  


//TODO
typedef struct UpdateInterval {
   double logicUpdateInterval;
   double spinUpdateInterval;
   double communicationUpdateInterval;
} UpdateInterval;  


typedef struct PIDparams {
   double Kp;
   double Ki;
   double Kd;
} PIDparams;


Geometry get_geometry();
void set_geometry(Geometry geometry);

PIDparams get_left_motor_pid_params();
void set_left_motor_pid_params(PIDparams params);

PIDparams get_right_motor_pid_params();
void set_right_motor_pid_params(PIDparams params);

void init_all_params();

#endif

