

#include "clamp_servos.h"



uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d)
{
 uint32_t clock = 125000000;
 uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
 if (divider16 / 16 == 0)
    divider16 = 16;
 uint32_t wrap = clock * 16 / divider16 / f - 1;
 pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
 pwm_set_wrap(slice_num, wrap);
 pwm_set_chan_level(slice_num, chan, wrap * d / 360);
 return wrap;
}
bool dance_mode = false;
void open_clamps(){
    set_clamp_right_angle_and_enable(1, RIGHT_OPEN_ANGLE);
    set_clamp_left_angle_and_enable(1, LEFT_OPEN_ANGLE);
    dance_mode = false;
}

void close_clamps(){
    set_clamp_right_angle_and_enable(1, RIGHT_CLOSE_ANGLE);
    set_clamp_left_angle_and_enable(1, LEFT_CLOSE_ANGLE);
    dance_mode = false;
}

void activate_dance_mode(){
    dance_mode = true;
}

int dance_state = 0;
long last_dance_time = 0;

void servos_dance_update(){
    if (dance_mode){

        if (last_dance_time + 350 > millis()){
            return;
        }
        last_dance_time = millis();

        switch(dance_state){
            case 0:
                set_clamp_right_angle_and_enable(1, RIGHT_CLOSE_ANGLE);
                set_clamp_left_angle_and_enable(1, LEFT_OPEN_ANGLE);
            break;
            case 1:
                set_clamp_right_angle_and_enable(1, RIGHT_OPEN_ANGLE);
                set_clamp_left_angle_and_enable(1, LEFT_CLOSE_ANGLE);
            break;
            case 2:
                set_clamp_right_angle_and_enable(1, RIGHT_CLOSE_ANGLE);
                set_clamp_left_angle_and_enable(1, LEFT_CLOSE_ANGLE);
            break;
            case 3:
                set_clamp_right_angle_and_enable(1, RIGHT_OPEN_ANGLE);
                set_clamp_left_angle_and_enable(1, LEFT_OPEN_ANGLE);
            break;
        }
        dance_state++;
        if (dance_state>4){
            dance_state =0;
        }
    }
}

void init_clamp_right(){

    uint slice_num = pwm_gpio_to_slice_num(PIN_CLAMP_RIGHT);
    uint channel = pwm_gpio_to_channel(PIN_CLAMP_RIGHT);

    gpio_set_function(PIN_CLAMP_RIGHT, GPIO_FUNC_PWM);

}

void set_clamp_right_angle_and_enable(int enable, int angle){
    uint slice_num = pwm_gpio_to_slice_num(PIN_CLAMP_RIGHT);
    uint channel = pwm_gpio_to_channel(PIN_CLAMP_RIGHT);

    if (enable == 1){
        pwm_set_freq_duty(slice_num, channel, 50, angle+90);
        pwm_set_enabled(slice_num, true);

    }else{
        pwm_set_clkdiv_int_frac(slice_num, 100, 10);
        pwm_set_wrap(slice_num, 0);
        pwm_set_chan_level(slice_num, channel, 100);
        pwm_set_enabled(slice_num, false);
    }
}


void init_clamp_left(){

    uint slice_num = pwm_gpio_to_slice_num(PIN_CLAMP_LEFT);
    uint channel = pwm_gpio_to_channel(PIN_CLAMP_LEFT);

    gpio_set_function(PIN_CLAMP_LEFT, GPIO_FUNC_PWM);

}

void set_clamp_left_angle_and_enable(int enable, int angle){
    uint slice_num = pwm_gpio_to_slice_num(PIN_CLAMP_LEFT);
    uint channel = pwm_gpio_to_channel(PIN_CLAMP_LEFT);

    if (enable == 1){
        pwm_set_freq_duty(slice_num, channel, 50, angle+90);
        pwm_set_enabled(slice_num, true);
    }else{
        pwm_set_clkdiv_int_frac(slice_num, 0xEE, 0x0);
        pwm_set_wrap(slice_num, 0);
        pwm_set_chan_level(slice_num, channel, 100);
        //pwm_set_enabled(slice_num, false);
    }
}