'''
    An example to demonstrate implementation of a PID controller to a line follower robot
    Implemented in micropython with ESP32
    the robot has 3[left, center, right] IR sensors and and 2 dc motors 

'''
from machine import *
from controller import PIDController #import PID
from time import ticks_ms, ticks_diff, sleep_ms

#some global variables
base_speed=100 #this should be adjusted based on the motor specifications
# initialize sensor pins as input
IR1=Pin(4, Pin.IN)
IR2=Pin(5, Pin.IN)
IR3=Pin(6, Pin.IN)

#initialize motor output pins with PWM
m1=PWM(Pin(35),frequency=1000) #check for the best frequency for the motors to be used
m2=PWM(Pin(36),frequency=1000)

#read the sensor values
sensor_values=[IR1.value(),IR2.value(),IR3.value()]

#initialize PID
PID=PIDController(kp=1,ki=0.1,kd=0.2) # should adjust the gain values based on tests

#compute the error(control) signal as a weighted sum
def compute_error(sensor_values):
    weights=[1,0,-1] #For left, center and right
    active=sum(sensor_values)
    if active==0: #if no line is detected
        return 0
    else:
        weighted_sum=sum(w*s for w,s in zip(weights,sensor_values))
        return weighted_sum/active
    
#drive the motors based on the corrected base speeds
def drive_motors(base_speed,correction):
    left_speed=base_speed-correction
    right_speed=base_speed+correction
    
    #clamp the duty cycle between 0-1023
    left_speed = min(max(int(left_speed), 0), 1023)
    right_speed = min(max(int(right_speed), 0), 1023)
    
    #assign corrected motor speeds
    m1.duty(left_speed)
    m2.duty(right_speed)
    
#main loop

while True:
    now = ticks_ms()
    dt = ticks_diff(now, last_time) / 1000  # Convert to seconds
    last_time = now
    
    error = compute_error(sensor_values)
    #call the update method from the pid class
    correction = PID.update(error, dt)

    drive_motors(base_speed, correction)
    sleep_ms(10)
    
