

#include "motor.h"
#include "communication.h"
#include "encoder.h"
#include "structs.h"
#include "security.h"
#include "odometry.h"
#include "regulator.h"
#include "params.h"
#include "clamp_servos.h"

void init_mccd(){

    delay(100);

    init_all_params();

    init_communication();


    init_motor_right(MODE_SLOW_DECAY, 1000);
    init_motor_left(MODE_SLOW_DECAY, 1000);

    init_encoders();
    init_regulator();

    init_clamp_left();
    init_clamp_right();

}



void start_mccd(){

    int dtLogic = 0;
    int dtCom = 0;
    int dtSpin = 0;
    int dtCheckAlive = 0;


    int logicUpdateInterval = 10;
    int spinUpdateInterval = 25;
    int communicationUpdateInterval = 20;
    int checkAliveUpdateInterval = 1000;

    int lastTime = millis();

    int elapsedTime = 0;
    int time;

    
    while(1){

        if (dtCom >= communicationUpdateInterval)
        {
            publish_status(get_status());
            publish_odom(get_position());
            dtCom = 0;
        }

        
        if (dtLogic >= logicUpdateInterval)
        {
            uint32_t dt = dtLogic;

            update_odometer(dt);
            //update_security(dt);
            update_regulator(dt);

            dtLogic = 0;
        }
        
        
        if (dtSpin >= spinUpdateInterval)
        {
            ros_spin_once_communication();
            servos_dance_update();
            dtSpin = 0;
        }

        if (dtCheckAlive >= checkAliveUpdateInterval)
        {
            check_and_reset_for_ros_agent_connected();
            dtCheckAlive = 0;
        }
        


        time = millis();
        elapsedTime = time - lastTime;
        lastTime = time;


        dtLogic += elapsedTime;
        dtCom += elapsedTime;
        dtSpin += elapsedTime;
        dtCheckAlive += elapsedTime;

    }
    
    
}