 /*
  A simple implementation of a PID controller on a line follower robot with 3 IR sensors and L298N motor driver  
*/
#include "controller.h"

// IR sensor pins
#define IR_LEFT    2
#define IR_CENTER  3
#define IR_RIGHT   4

// L298N motor driver pins
#define ENA 5  //motor 1 PWM
#define IN1 6
#define IN2 7

#define ENB 9  //motor 2 PWM
#define IN3 10
#define IN4 11

// PID structure
gains pid;

void setup() {
  Serial.begin(9600);

  // Sensor setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize PID (tune kp, ki, kd)
  init_pid(&pid, 1.0, 0.2, 0.1); // start with just P and D and adjust to suite the system response
}

void loop() {
  int error = getLineError();
  float dt = 0.05; // 50ms
  float correction = update(&pid, (float)error, dt);

  int base_speed = 150; //should be adjusted based on motor type
  int left_speed = base_speed - correction;
  int right_speed = base_speed + correction;

  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  moveMotors(left_speed, right_speed);

  delay(50); // 50ms cycle
}

// Reads sensor state and returns a signed error
int getLineError() {
  int left = digitalRead(IR_LEFT);
  int center = digitalRead(IR_CENTER);
  int right = digitalRead(IR_RIGHT);

//check how far the robot is from the line
  if (center == 1 && left == 0 && right == 0) return 0;    // on track
  else if (left == 1 && center == 1 && right == 0) return 1; //drifting left slightly turn right
  else if (left == 1 && center == 0 && right == 0) return 2; // sharp right turn
  else if (right == 1 && center == 1 && left == 0) return -1; // drifting right slightly turn left
  else if (right == 1 && center == 0 && left == 0) return -2; //sharp left turn
  else if (left == 1 && center == 1 && right == 1) return 0; // continue straight or we can custom instructions treat it as a T-junction
  else return 0; // Line lost or ambiguous
  
}

/* sensor pattern summary
| Sensor Pattern | Meaning                   | PID Error   | Action                       |
| -------------- | ------------------------- | ----------- | ---------------------------- |
| 0 1 0          | On line                   | 0           | Go straight                  |
| 1 1 0          | Slightly right of line    | +1          | Slight right turn            |
| 1 0 0          | Far right of line         | +2          | Sharp right turn             |
| 0 1 1          | Slightly left of line     | -1          | Slight left turn             |
| 0 0 1          | Far left of line          | -2          | Sharp left turn              |
| 1 1 1          | Ambiguous or intersection | 0 (default) | Go straight or custom action |

*/

// Drives both motors with given speeds
void moveMotors(int leftSpeed, int rightSpeed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}


