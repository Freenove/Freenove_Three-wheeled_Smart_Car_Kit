/*
  Sketch    Exploration mode sketch for car
  Platform  Freenove Three-wheeled Smart Car Kit for Arduino
  Author    Ethan Pan @ Freenove (http://www.freenove.com)
  Date      2016/8/13
  Brief     This sketch is used to achieve exploration mode for Freenove Three-wheeled
            Smart Car Kit for Arduino. This sketch needs to be uploaded to the car.
            The car needs to install NFR24L01 wireless communication module,
            and install HC-SR04 ultrasonic ranging module in the pan tilt.
  Copyright Copyright © Freenove (http://www.freenove.com)
  License   Creative Commons Attribution ShareAlike 3.0
            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
*/

// NRF24L01
#include <SPI.h>
#include "RF24.h"
RF24 radio(9, 10);                // define the object to control NRF24L01
const byte addresses[6] = "Free1";// define communication address which should correspond to remote control
// servo
#include <Servo.h>
Servo dirServo;                  // define servo to control turning of smart car
const int dirServoPin = 2;        // define pin for signal line of the last servo
const float dirServoOffset = 0;   // define a variable for deviation(degree) of the last servo
Servo ptServo;                   // define servo to control the pan tilt
const int ptServoPin = 3;         // define pin for signal line of the last servo
const float ptServoOffset = 0;    // define a variable for deviation(degree) of the last servo
// motor
const int dirAPin = 7,            // define pin used to control rotational direction of motor A
          pwmAPin = 6,            // define pin for PWM used to control rotational speed of motor A
          dirBPin = 4,            // define pin used to control rotational direction of motor B
          pwmBPin = 5,            // define pin for PWM used to control rotational speed of motor B
          snsAPin = A0,           // define pin for detecting current of motor A
          snsBPin = A1;           // define pin for detecting current of motor B
const byte motorSpeed = 128;      // define motor speed(0-255)
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
// wireless communication
int dataRead[4];                  // define array used to save the read data
int dataWrite[ptScanTimes + 2];   // define array used to save the write data
// multi task time allocation
const int scanInterval = 30;      // define the interval(ms) of each scan

void setup() {
  // serial port
  Serial.begin(115200);
  // NRF24L01
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openWritingPipe(addresses);   // open delivery channel
  radio.openReadingPipe(1, addresses);// open delivery channel
  radio.startListening();             // start monitoring
  // servo
  dirServo.attach(dirServoPin); // attaches the servo on dirServoPin to the servo object
  ptServo.attach(ptServoPin);   // attaches the servo on ptServoPin to the servo object
  // motor
  pinMode(dirAPin, OUTPUT);     // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);     // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);     // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);     // set pwmBPin to output mode
  // ultrasonic ranging module
  pinMode(trigPin, OUTPUT);     // set trigPin to output mode
  pinMode(echoPin, INPUT);      // set echoPin to input mode
  // initialize the array
  for (int i = 0; i < ptScanTimes; i++)
    scanDistance[i] = maxDistance;
}

void loop()
{
  unsigned long ms = millis();
  static unsigned long lastScan = ms;

  // scan distance of obstacle
  // this task will be executed every scanInterval ms
  if (ms - lastScan >= scanInterval) {
    scan();
    lastScan = ms / scanInterval * scanInterval;
  }

  // save scan distance
  for (int i = 0; i < ptScanTimes; i++)
    dataWrite[i] = scanDistance[i];

  // save motor current
  float iMotorA = analogRead(snsAPin) * 5.0 / 1024 / 30 / 0.05; // motor A current(A)
  float iMotorB = analogRead(snsBPin) * 5.0 / 1024 / 30 / 0.05; // motor B current(A)
  iMotorA *= 100;
  iMotorB *= 100;
  dataWrite[ptScanTimes] = iMotorA;
  dataWrite[ptScanTimes + 1] = iMotorB;

  // read radio data
  if ( radio.available()) {             // if receive the data
    while (radio.available()) {         // read all the data
      radio.read(dataRead, sizeof(dataRead));   // read data
    }
    delay(5);

    // write radio data
    radio.stopListening();              // stop monitoring
    radio.write(dataWrite, sizeof(dataWrite));  // write data
    radio.startListening();             // start monitoring

    // calculate the angle of turning
    int dirServoDegree = map(dataRead[2], 0, 1023, 135, 45) - (dataRead[0] - 512) / 25;
    // calculate speed of moving
    int motorSpd = dataRead[3] - 512 + (dataRead[1] - 512) / 10;
    bool motorDir = motorSpd > 0 ? BACKWARD : FORWARD;
    motorSpd = abs(constrain(motorSpd, -512, 512));
    motorSpd = map(motorSpd, 0, 512, 0, motorSpeed);

    // control the turning and moving of the car
    ctrlCar(dirServoDegree, motorDir, motorSpd);
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

