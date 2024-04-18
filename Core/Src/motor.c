#include "motor.h"
#include "stdio.h"

motor_p motor_left;
motor_p motor_right;

void motor_init(){
    motor_left.encoder_val = 0;
    motor_right.encoder_val = 0;
    motor_left.current_speed = 0;
    motor_right.current_speed = 0;
    motor_left.encoder_last_val = 0;
    motor_right.encoder_last_val = 0;
}