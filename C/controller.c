/*
This is the main module contains the actual fucntions for the PID 
*/
#include "controller.h"
struct gains {
    //define all the gains in here
    int kp;
    int ki;
    int kd;
    int integral;
    int prev_error;
};

int update(struct gains *pid,int error, int dt){
    /*initialize the structure as a pointer for use in this fuction to avoid breaking of the pid logic
    such that the values of the derivative and the integral parts will be saved on each iteration */
    //proportional gain

    int P= pid->kp* error;

    //integral gain
    pid->integral += error * dt;
    int I = pid->ki * pid->integral;

    //derivative gain
    int derivative = (error - pid->prev_error) / dt;
    int D = pid->kd * derivative;

    pid->prev_error = error;

    return P+I+D;

}