

#include "encoder.h"
#include "hardware/pio.h"
#include "quadrature_encoder_pio.h"

int32_t ticks_left = 0;
int32_t ticks_right = 0;

const uint sm_left = 0;
const uint sm_right = 1;

void init_encoders(){
    const uint PIN_AB_LEFT = PIN_CHAN_A_L;
    const uint PIN_AB_RIGHT = PIN_CHAN_A_R;

    PIO pio = pio0;

    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm_left, offset, PIN_AB_LEFT, 0);
    quadrature_encoder_program_init(pio, sm_right, offset, PIN_AB_RIGHT, 0);
}

void update_encoders_ticks(){
    quadrature_encoder_get_count(pio0, sm_left, sm_right, &ticks_left, &ticks_right);
}


int32_t get_ticks_encoder_left() 
{
    return ticks_left;
}

int32_t get_ticks_encoder_right() 
{
    return ticks_right;
}
