#ifndef PINS_HPP
#define PINS_HPP


// RIGHT MOTOR
#define PIN_PHASE_R       3
#define PIN_MODE_R        2
#define PIN_ENABLE_R      4
#define PIN_OCL_R         6
#define PIN_VREF_R        5
#define PIN_CURR_SENSE_R  27
#define PIN_PHASE_PWM_MODE_R PWM_CHAN_B



// LEFT MOTOR   
#define PIN_PHASE_L       8
#define PIN_MODE_L        7
#define PIN_ENABLE_L      9
#define PIN_OCL_L         11
#define PIN_VREF_L        10
#define PIN_CURR_SENSE_L  26
#define PIN_PHASE_PWM_MODE_L PWM_CHAN_A




// RIGHT ENCODER
#define PIN_CHAN_A_R 20
#define PIN_CHAN_B_R PIN_CHAN_A_R+1
#define PIN_CHAN_Z_R 22


// LEFT ENCODER
#define PIN_CHAN_A_L 16
#define PIN_CHAN_B_L PIN_CHAN_A_L+1
#define PIN_CHAN_Z_L 18


// FDC
#define PIN_FDC_FR 12
#define PIN_FDC_FL 15


// TMP 2023/2023 only
#define PIN_CLAMP_RIGHT PIN_FDC_FR
#define PIN_CLAMP_LEFT PIN_FDC_FL





#endif