 /*
  A simple implementation of a PID controller on a line follower robot with 3 IR sensors 
  with two L298N motor driver  
*/
#include <stdio.h>
#include "controller.h"

void setup() {
  Serial.begin(9600);
  //IR sensors
  PinMode(2,INPUT);
  PinMode(3,INPUT);
  PinMode(4,INPUT);

  //L298N motor driver pins initialize pins with PWM support
  PinMode(6,OUTPUT);
  PinMode(9,OUPUT);
  PinMode(10,OUTPUT);
  PinMode(11,OUTPUT);
}

void loop() {
  //run the functions
  int error=compute_error();
  pid_correction=pid.update(error, dt)
  drive_robot();
}

void compute_error(){
  //initialize the PID 
  struct gains pid = { .kp = 1, .ki = 0.1, .kd = 0.2, .integral = 0, .prev_error = 0 };

  //initialize pointers to the sensor readings and the weights 
  int *sensors;
  int *weights;

  //get sensor readings
  int sensors[]={digitalRead(2),digitalRead(3),digitalRead(4)};
  // assign weights for the left center and right sensor readings
  int weights[]={1,0,-1};

  int len= sizeof(sensors) / sizeof(sensors[0])
  //sum the sensor values to check if a line has been detected 
  
  for(int i=0; i<len; i++){
    int active += sensors[i];
    int weighted_sum=0;
    weighted_sum += sensors[i] * weights[i];
  }
  // check a line has been detected 
  if (active==0){
    return 0;
  }
  else {
    return weighted_sum/active;
  }
}

void drive_robot(int base_speed, int pid_correction){
  //Implement a PWM control on the motor pins 
  
  for(int i=0;  )
}
