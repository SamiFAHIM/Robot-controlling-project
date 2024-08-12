#include <Arduino.h>
#include "clamp_servos.h"
#include "pins.h"

#define ROS_VERSION
#ifdef ROS_VERSION

    #include "mccd.h"

    void setup() {
        Serial.begin(115200);

        init_mccd();

        start_mccd();
    }

    void loop() {}

#else

    #include "mccd_rosless.h"

    void setup() {
        Serial.begin(115200);

        init_mccd_rosless();

        start_mccd_rosless();
    }

    void loop() {}

#endif