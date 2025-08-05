/*
This is the main module contains the actual fucntions for the PID 
*/
#include "controller.h"

//initialize the PID 
void init_pid(gains *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float update(gains *pid,float error, float dt){
    /*initialize the structure as a pointer for use in this fuction to avoid breaking of the pid logic
    such that the values of the derivative and the integral parts will be saved on each iteration */

    //avoid division by zero in the derivative term 
    if (dt <= 0.0f) dt = 1e-3f;

    //proportional gain
    float p= pid->kp* error;

    //integral gain
    pid->integral += error * dt;

    //clamping the gains to avoid windup 
    float max_integral = 100.0; // Choose the most appropriate value based on application
    if (pid->integral > max_integral)
        pid->integral = max_integral;
    else if (pid->integral < -max_integral)
        pid->integral = -max_integral;
        
    float i = pid->ki * pid->integral;

    //derivative gain
    float derivative = (error - pid->prev_error) / dt;
    float d = pid->kd * derivative;

    pid->prev_error = error;

    return p+i+d;

}