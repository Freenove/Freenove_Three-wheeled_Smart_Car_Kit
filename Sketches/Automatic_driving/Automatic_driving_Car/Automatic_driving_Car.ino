/*
  Sketch    Automatic driving mode sketch for car
  Platform  Freenove Three-wheeled Smart Car Kit for Arduino
  Author    Ethan Pan @ Freenove (http://www.freenove.com)
  Date      2016/8/13
  Brief     This sketch is used to achieve automatic driving mode for Freenove Three-wheeled
            Smart Car Kit for Arduino. This sketch needs to be uploaded to the car.
            The car needs to install HC-SR04 ultrasonic ranging module in the pan tilt.
  Copyright Copyright © Freenove (http://www.freenove.com)
  License   Creative Commons Attribution ShareAlike 3.0
            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
*/

// servo
#include <Servo.h>
Servo dirServo;                  // define servo to control turning of smart car
const int dirServoPin = 2;        // define pin for signal line of the last servo
const float dirServoOffset = 6;   // define a variable for deviation(degree) of the last servo
Servo ptServo;                   // define servo to control the pan tilt
const int ptServoPin = 3;         // define pin for signal line of the last servo
const float ptServoOffset = 5;    // define a variable for deviation(degree) of the last servo
// motor
const int dirAPin = 7,            // define pin used to control rotational direction of motor A
          pwmAPin = 6,            // define pin for PWM used to control rotational speed of motor A
          dirBPin = 4,            // define pin used to control rotational direction of motor B
          pwmBPin = 5;            // define pin for PWM used to control rotational speed of motor B
const byte motorSpeed = 96;       // define motor speed(0-255)
#define FORWARD LOW
#define BACKWARD HIGH
// ultrasonic ranging
const int trigPin = A4;           // define Trig pin for ultrasonic ranging module
const int echoPin = A5;           // define Echo pin for ultrasonic ranging module
const float maxDistance = 200;    // define the range(cm) for ultrasonic ranging module
const float soundVelocity = 340;  // define sound velocity = 340 m/s
const float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module
// scan distance of obstacle
const int ptScanInterval = 10;    // define the interval(degree) of each scan
const int ptScanTimes = 11;       // define the number of scan times per cycle
byte scanDistance[ptScanTimes];   // define an array to save scan data
// multi task time allocation
const int scanInterval = 30;      // define the interval(ms) of each scan
const int moveInterval = 1000;    // define the interval(ms) of each moving car

void setup() {
  // serial port
  Serial.begin(115200);
  // servo
  dirServo.attach(dirServoPin);   // attaches the servo on dirServoPin to the servo object
  ptServo.attach(ptServoPin);     // attaches the servo on ptServoPin to the servo object
  // motor
  pinMode(dirAPin, OUTPUT);       // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);       // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);       // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);       // set pwmBPin to output mode
  // ultrasonic ranging module
  pinMode(trigPin, OUTPUT);       // set trigPin to output mode
  pinMode(echoPin, INPUT);        // set echoPin to input mode
  // initialize the array
  for (int i = 0; i < ptScanTimes; i++)
    scanDistance[i] = maxDistance;
}

void loop()
{
  unsigned long ms = millis();
  static unsigned long lastScan = ms;
  static unsigned long lastMove = ms;

  // scan distance of obstacle
  // this task will be executed every scanInterval ms
  if (ms - lastScan >= scanInterval) {
    scan();
    lastScan = ms / scanInterval * scanInterval;
  }
  // move the car according to scan distance
  // this task will be executed every moveInterval ms
  if (ms - lastMove >= moveInterval) {
    moveCar();
    lastMove = ms / moveInterval * moveInterval;
  }
}

void moveCar()
{
  int angle, distance;
  // get nearst obstacle's angle and distance
  getNearest(&angle, &distance);
  // according nearest obstacle's angle and distance to move the car
  if (distance < 20) {     // if the obstacle distance is too close, back the car
    if (angle < 90)        // choose turning direction when back
      ctrlCar(135, BACKWARD, motorSpeed * 2);
    else
      ctrlCar(45, BACKWARD, motorSpeed * 2);
  }
  else if (distance < 50) {// if there is obstacle, turn the forward direction
    if (angle < 90)        // choose turning direction when move forward
      ctrlCar(135, FORWARD, motorSpeed);
    else
      ctrlCar(45, FORWARD, motorSpeed);
  }
  else {                   // if there is no obstacle, move forward
    ctrlCar(90, FORWARD, motorSpeed);
  }
}

void scan() {
  static byte ptAngle = 90;
  static bool pdDir = true;
  // calculate dependent variables
  int minAngle = (180 - ptScanInterval * (ptScanTimes - 1)) / 2;
  int maxAngle = 180 - minAngle;
  int index = (ptAngle - minAngle) / ptScanInterval;
  // detect the current distance from obstacle
  scanDistance[index] = getDistance();
  // send measured distance to the serial port
  Serial.print(scanDistance[index]);
  if (ptAngle >= maxAngle) {
    pdDir = false;
    Serial.print("; ");
  }
  else if (ptAngle <= minAngle) {
    pdDir = true;
    Serial.print("\n");
  }
  else
    Serial.print(", ");
  // turn pan tilt to the next position
  pdDir ? ptAngle += ptScanInterval : ptAngle -= ptScanInterval;
  ptServo.write(ptAngle + ptServoOffset);
}

void getNearest(int *angle, int *distance) {
  *distance = maxDistance;
  *angle = 90;
  // calculate nearst obstacle's angle and distance
  for (int i = 0; i < ptScanTimes; i++) {
    if (scanDistance[i] < *distance) {
      *distance = scanDistance[i];
      *angle = (180 - ptScanInterval * (ptScanTimes - 1)) / 2 + i * ptScanInterval;
    }
  }
}

float getDistance() {
  unsigned long pingTime; // save the high level time returned by ultrasonic ranging module
  float distance;         // save the distance away from obstacle
  // set the trigPin output 10us high level to make the ultrasonic ranging module start to measure
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // get the high level time returned by ultrasonic ranging module
  pingTime = pulseIn(echoPin, HIGH, rangingTimeOut);
  // calculate the obstacle distance
  if (pingTime != 0) {  // if the measure is not overtime
    distance = pingTime * soundVelocity / 2 / 10000;  // calculate the obstacle distance(cm) according to the time of high level returned
    return distance;    // return distance(cm)
  }
  else                  // if the measure is overtime
    return maxDistance; // returns the maximum distance(cm)
}

void ctrlCar(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);
  digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
}

