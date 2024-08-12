
#include "odometry.h"
#include "encoder.h"
#include "params.h"
#include "Math.hpp"

Odom current_odom;
int32_t odom_last_tick_left;
int32_t odom_last_tick_right;

double left_wheel_speed;
double right_wheel_speed;
double linear_speed;
double angular_speed;


Odom get_position(){
    return current_odom;
}


void set_position(Odom new_odom){
    current_odom.x = new_odom.x;
    current_odom.y = new_odom.y;
    current_odom.theta = new_odom.theta;
}



void update_odometer(uint32_t dt){
    update_encoders_ticks();



    int32_t tick_right = get_ticks_encoder_right();
    int32_t tick_left = get_ticks_encoder_left();

    int32_t d_tick_left = tick_left - odom_last_tick_left;
    int32_t d_tick_right = tick_right - odom_last_tick_right;

    // Small displacement of the left wheel, in meters
    double dl = get_geometry().left_encoder_conversion_factor * d_tick_left;
    // Small displacement of the right wheel, in meters
    double dr = get_geometry().right_encoder_conversion_factor * d_tick_right;

    

    


    double dDistance = (dr + dl) / 2;
    double dTheta = (dr - dl) / get_geometry().distance_between_encoders;

    

    current_odom.theta = std::fmod(current_odom.theta + dTheta, TWO_PI);
    current_odom.x += std::cos(current_odom.theta) * dDistance;
    current_odom.y += std::sin(current_odom.theta) * dDistance;

    

    odom_last_tick_left = tick_left;
    odom_last_tick_right = tick_right;

    
    left_wheel_speed = dl / dt * 1000;
    right_wheel_speed = dr / dt * 1000;
    linear_speed = dDistance / dt * 1000;
    angular_speed = dTheta / dt * 1000;


        /*
    Serial.print("l:");

    Serial.print(dl);
    Serial.print( "  -  ");
    Serial.print(d_tick_left);
    Serial.print( "  -  ");
    Serial.print(left_wheel_speed);
    Serial.print(" | r:");

    Serial.print(dr);
    Serial.print( "  -  ");
    Serial.print(d_tick_right);
    Serial.print( "  -  ");
    Serial.print(right_wheel_speed);
    Serial.println();
    */

}


double get_left_wheel_speed(){
    return left_wheel_speed;
}

double get_right_wheel_speed(){
    return right_wheel_speed;
}