#ifndef __PID_H__
#define __PID_H__
#include "main.h"

typedef struct{
    float SetVoltage;
    float ActualVoltage;
    float err;
    float err_last;
    float Kp,Ki,Kd;
    float result;
    float voltage;
    float integral;
}pid_p;

void PID_init( void);
float PID_realize( float v, float v_r);

#endif