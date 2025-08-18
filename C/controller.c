/*
This is the main module contains the actual fucntions for the PID 
*/
#include "controller.h"

//initialize the PID 
void init_pid(gains *pid, float kp, float ki, float kd, float max_integral, float tau) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral=max_integral;
    pid->tau=tau; //the time constant for a low pass filter
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative=0.0f; //initializing the filtered derivative
}

//Reset the PID values at startup 
void reset_pid(gains *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
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

    // Integral term with clamping. The clamping value is configurable based on system requirements
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float i = pid->ki * pid->integral;

    // Derivative gain with a low pass filter to reduce high freq noise that can cause instability
    float derivative = (error - pid->prev_error) / dt;
    // Apply low-pass filter: 
    // filtered_derivative = (tau * prev_derivative + dt * raw_derivative) / (tau + dt)
    float alpha = dt / (pid->tau + dt);  // Filter coefficient
    float filtered_derivative = alpha * derivative + (1.0f - alpha) * pid->prev_derivative;
    pid->prev_derivative = filtered_derivative;  // Store for next iteration

    float d = pid->kd * filtered_derivative;
    pid->prev_error = error;

    return p+i+d;

}