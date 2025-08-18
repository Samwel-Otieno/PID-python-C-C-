 /*
  Implementation of a PID controller on a simple line follower robot with 3 IR sensors and L298N motor driver  
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

// Constants
const float dt = 0.05f;       // 50ms control loop
const int base_speed = 150;   // Base motor speed (0-255)
const float MAX_INTEGRAL = 50.0f;  // Anti-windup limit
const float TAU = 0.1f;       // Low-pass filter time constant (adjust for noise)
// Logging interval (milliseconds)
const unsigned long LOG_INTERVAL = 100;  // Log every 100ms to avoid flooding
unsigned long lastLogTime = 0;

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
  init_pid(&pid, 1.0f, 0.0, 0.1f, MAX_INTEGRAL,TAU); // start with just P and D and adjust to suite the system response
  // Header for CSV-style logging
  Serial.println("Time(ms),Error,P-Term,I-Term,D-Term,LeftSpeed,RightSpeed,SensorPattern");
} 

void loop() {
  unsigned long currentTime = millis();
  int error = getLineError();
  float dt = 0.05; // 50ms
  float correction = update(&pid, (float)error, dt);

  int base_speed = 150; //should be adjusted based on motor type
  int left_speed = base_speed - correction;
  int right_speed = base_speed + correction;

  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  moveMotors(left_speed, right_speed);

  // Optional: Reset PID on large error (e.g., line lost)
  if (abs(error) >= 2) {
    reset_pid(&pid);  // Reset integral/derivative history this prevents integral windup during sharp turns/recovery.
    Serial.println("PID Reset!");  // Log reset events
  }
    // Log data at fixed intervals
  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    logData(currentTime, error, left_speed, right_speed);
    lastLogTime = currentTime;
  }

  delay((int)(dt * 1000));  // Convert DT to milliseconds for cleaner timing
}

// Log data to Serial Monitor
void logData(unsigned long time, int error, int leftSpeed, int rightSpeed) {
  // Read sensor states
  int left = digitalRead(IR_LEFT);
  int center = digitalRead(IR_CENTER);
  int right = digitalRead(IR_RIGHT);

  String sensorPattern = String(left) + String(center) + String(right);

  // output data in a csv format
  Serial.print(time); Serial.print(",");
  Serial.print(error); Serial.print(",");
  Serial.print(pid.kp * error); Serial.print(",");  // P-Term
  Serial.print(pid.ki * pid.integral); Serial.print(",");  // I-Term
  Serial.print(pid.kd * pid.prev_derivative); Serial.print(",");  // D-Term
  Serial.print(leftSpeed); Serial.print(",");
  Serial.print(rightSpeed); Serial.print(",");
  Serial.println(sensorPattern);
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


