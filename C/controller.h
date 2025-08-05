/*
This is the header file to be imported as #include <controller.h>
module definition and initialization
*/
#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
    //define all the gains in here
    //use floating point data to accomodate slowly changing errors
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} gains;

void init_pid(gains *pid, float kp, float ki, float kd);

float update(gains *pid, float error, float dt);

#endif