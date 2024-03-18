#include "info.h"
#include "motor.h"


int get_motor_info(Motor* motor, char *motor_info) {
    char buf[64];
    
    sprintf(motor_info,"%s, mps: %f, rps: %f", 
        motor->name,
        motor->motorStats.velocity,
        motor->motorStats.rps);
    
    return 1;
}
