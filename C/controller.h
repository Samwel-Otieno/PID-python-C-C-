/*
This is the header file to be imported as #include <controller.h>
module definition and initialization
*/
#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
    //define all the gains in here
    //use floating point data to accomodate slowly changing errors
    float kp,ki,kd;
    float integral;
    float prev_error;
    float max_integral;
    float prev_derivative;  // Stores the last filtered derivative value
    float tau;             // Time constant for the low-pass filter (smaller = more filtering)
} gains;

void init_pid(gains *pid, float kp, float ki, float kd, float max_integral, float tau);

float update(gains *pid, float error, float dt);
void reset_pid(gains *pid);  // Reset all PID states

#endif