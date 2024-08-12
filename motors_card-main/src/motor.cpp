
#include "motor.h"

uint8_t is_motor_left_enabled;
uint8_t is_motor_right_enabled;

void set_speed_motor_left(int dir, int speed){
    uint slice_num = pwm_gpio_to_slice_num(PIN_PHASE_L);

    if (speed >= 500){
        speed = 500;
    }

    if (speed <= 0 && dir != DIR_SIGNED){
        speed = 0;
    }else if(speed <= -500){
        speed = -500;
    }
    int stop_speed = 500;

    switch (dir){
        case DIR_STOP:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_L, stop_speed);
            break;
        case DIR_BACKWARD:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_L, stop_speed-speed);
            break;
        case DIR_FORWARD:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_L, stop_speed+speed);
            break;
        case DIR_SIGNED:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_L, stop_speed+speed);
            break;
    }
}

void enable_pwm_motor_left(int enabled){
    if (is_motor_left_enabled && enabled){
        return;
    }
    uint slice_num = pwm_gpio_to_slice_num(PIN_PHASE_L);

    gpio_set_function(PIN_PHASE_L, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.f);
    pwm_init(slice_num, &config, true);

    pwm_set_wrap(slice_num, 1000);
    pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_L, 500);      // 25% duty cycle
    pwm_set_enabled(slice_num, true);


    set_speed_motor_left(DIR_STOP, 0);

    digitalWrite(PIN_ENABLE_L, enabled);

    is_motor_left_enabled = enabled;

}

void init_motor_left(int decay_mode, int current_limit){
    pinMode(PIN_OCL_L, INPUT);
    pinMode(PIN_ENABLE_L, OUTPUT);
    pinMode(PIN_MODE_L, OUTPUT);
    digitalWrite(PIN_MODE_L, decay_mode);

    
    analogWrite(PIN_VREF_L, current_limit);
}


void set_speed_motor_right(int dir, int speed){
    uint slice_num = pwm_gpio_to_slice_num(PIN_PHASE_R);

    if (speed >= 500){
        speed = 500;
    }

    if (speed <= 0 && dir != DIR_SIGNED){
        speed = 0;
    }else if(speed <= -500){
        speed = -500;
    }

    int stop_speed = 500;
    switch (dir){
        case DIR_STOP:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_R, stop_speed);
            break;
        case DIR_BACKWARD:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_R, stop_speed-speed);
            break;
        case DIR_FORWARD:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_R, stop_speed+speed);
            break;
        case DIR_SIGNED:
            pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_R, stop_speed+speed);
            break;
    }
}

void enable_pwm_motor_right(int enabled){
    if (is_motor_right_enabled && enabled){
        return;
    }

    uint slice_num = pwm_gpio_to_slice_num(PIN_PHASE_R);

    gpio_set_function(PIN_PHASE_R, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.f);
    pwm_init(slice_num, &config, true);

    pwm_set_wrap(slice_num, 1000);
    pwm_set_chan_level(slice_num, PIN_PHASE_PWM_MODE_R, 500);      // 25% duty cycle
    pwm_set_enabled(slice_num, true);


    set_speed_motor_right(DIR_STOP, 0);

    digitalWrite(PIN_ENABLE_R, enabled);

    is_motor_right_enabled = enabled;
}


void init_motor_right(int decay_mode, int current_limit){
    pinMode(PIN_OCL_R, INPUT);
    pinMode(PIN_ENABLE_R, OUTPUT);
    pinMode(PIN_MODE_R, OUTPUT);
    digitalWrite(PIN_MODE_R, decay_mode);
    analogWrite(PIN_VREF_R, current_limit);
}
