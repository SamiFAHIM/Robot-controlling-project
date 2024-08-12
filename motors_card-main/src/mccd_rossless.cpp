#include "motor.h"
#include "encoder.h"
#include "structs.h"
#include "security.h"
#include "odometry.h"
#include "regulator.h"
#include "params.h"

void init_mccd_rosless(){
    Serial.println("init_mccd_rosless");

    init_all_params();

    init_motor_right(MODE_SLOW_DECAY, 1000);
    init_motor_left(MODE_SLOW_DECAY, 1000);

    init_encoders();
    init_regulator();

    Serial.println("init_mccd_rosless done");
}



void start_mccd_rosless(){
    Serial.println("start_mccd_rosless");

    int dtLogic = 0;
    int dtCom = 0;

    int logicUpdateInterval = 10;
    int communicationUpdateInterval = 20;

    int lastTime = millis();

    int elapsedTime = 0;
    int time;
    
    while(1){

        if(dtCom >= communicationUpdateInterval){
                        
            // put your communication code here
            //Serial.println("Hello");

            dtCom = 0;
        }

        if (dtLogic >= logicUpdateInterval)
        {
            uint32_t dt = dtLogic;

            update_odometer(dt);
            update_regulator(dt);

            dtLogic = 0;
        }

        time = millis();
        elapsedTime = time - lastTime;
        lastTime = time;


        dtLogic += elapsedTime;
        dtCom += elapsedTime;
    }

}