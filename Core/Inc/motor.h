#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h"

typedef struct{
    int16_t encoder_val;
    int16_t current_speed;
    int16_t encoder_last_val;
}motor_p;

void motor_init( void);

#endif