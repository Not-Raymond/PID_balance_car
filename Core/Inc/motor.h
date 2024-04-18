#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h"

typedef struct{
    uint16_t encoder_val;
    uint16_t current_speed;
    uint16_t encoder_last_val;
}motor_p;

void motor_init( void);

#endif