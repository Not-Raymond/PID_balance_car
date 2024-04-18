#include "pid.h"
#include "stdio.h"

pid_p pid;

void PID_init(){
    printf("PID_init begin \n");
    pid.SetVoltage = 0.0;
    pid.ActualVoltage = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;
    pid.voltage = 0.0;
    pid.integral = 0.0;
    pid.Kp = 0.0;
    pid.Ki = 0.0;
    pid.Kd = 0.0;
    printf("PID_init end \n");
}

float PID_realize( float v, float v_r){
    pid.SetVoltage = v;
    pid.ActualVoltage = v_r;
    pid.err = pid.SetVoltage - pid.ActualVoltage;
    pid.integral += pid.err;
    pid.result = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
    pid.err_last = pid.err;
    return pid.result;
}