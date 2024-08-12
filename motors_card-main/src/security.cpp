

#include "security.h"


uint32_t current_status;

uint32_t get_status(){
    return current_status;
}


uint32_t update_security(uint32_t dt){
    current_status = STATUS_OFF_OK;
}