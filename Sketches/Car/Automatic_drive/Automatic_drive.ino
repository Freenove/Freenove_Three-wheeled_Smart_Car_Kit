/*
  Automatic drive

  The sample code of Freenove Smart Car Shield for Arduino UNO, can make the smart car
  work in automatic driving mode.
  This code applies to Freenove Three-wheeled Smart Car. You need to install one-dimensional
  servo pan tilt for the smart car and install the ultrasonic ranging module in the pan tilt.

  created 2016/5/20
  by http://www.freenove.com
*/

#include <Servo.h>
Servo dirServo;          // define servo to control turning of smart car
int dirServoPin = 2;      // define pin for signal line of the last servo
float dirServoOffset = 6; // define a variable for deviation(degree) of the servo
Servo ptServo;           // define servo to control the pan tilt
int ptServoPin = 3;       // define signal pin for the servo of the pan tilt
float ptServoOffset = 5;  // define deviation(degree) of the servo of the pan tilt

int trigPin = A4;         // define Trig pin for ultrasonic ranging module
int echoPin = A5;         // define Echo pin for ultrasonic ranging module
float maxDistance = 200;  // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.
float soundVelocity = 340;// sound velocity = 340 m/s
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module

int dirAPin = 7,    // define pin used to control rotational direction of motor A
    pwmAPin = 6,    // define pin for PWM used to control rotational speed of motor A
    dirBPin = 4,    // define pin used to control rotational direction of motor B
    pwmBPin = 5;    // define pin for PWM used to control rotational speed of motor B

#define FORWARD LOW
#define BACKWARD HIGH

void setup() {
  dirServo.attach(dirServoPin);  // attaches the servo on dirServoPin to the servo object
  ptServo.attach(ptServoPin);    // attaches the servo on ptServoPin to the servo object

  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode

  pinMode(trigPin, OUTPUT); // set trigPin to output mode
  pinMode(echoPin, INPUT);  // set echoPin to input mode

  Serial.begin(9600); // Initialize serial port
}

void loop()
{
  byte barDistance = maxDistance; // save the minimum measured distance from obstacles
  byte barDegree;                 // save the minimum measured angel from obstacles
  byte distance;                  // save the current the measured distance from obstacles

  // define the initial scanning position servo of pan tilt
  ptServo.write(40 + ptServoOffset);
  delay(200);
  // start to scan distance. During this progress, we will get the distance and angle from the closest obstacle
  for (byte ptDegree = 40; ptDegree < 140; ptDegree += 10) {
    ptServo.write(ptDegree + ptServoOffset); // steer pan tilt to corresponding position
    delay(50);                // wait 50ms between pings (about 20 pingsc). 29ms should be the shortest delay between pings.
    distance = getDistance(); // detect the current distance from obstacle with angle of pan tilt stable

    // send measured distance to the serial port
    Serial.print(distance);
    Serial.print(", ");

    if (distance < barDistance) { // if the current measured distance is smaller than the previous one, save the data of current measured distance
      barDegree = ptDegree;       // save the measured angle
      barDistance = distance;     // save the measured distance
    }
  }
  Serial.println();

  // servo of pan tilt turns to default position
  delay(200);
  ptServo.write(90 + ptServoOffset);

  int spd = 128;  // set the speed(0-255) of smart car
  // 根据扫描的结果控制智能车的行动
  if (barDistance < 20) {     // if the obstacle distance is too close, reverse the travelling direction
    if (barDegree < 90)       // choose to reverse direction according to the angle with obstacle
      ctrlCar(135, BACKWARD, spd);  // control steering and reversing smart car
    else
      ctrlCar(45, BACKWARD, spd);   // control steering and reversing smart car
    delay(1500);              // reversing time
  }
  else if (barDistance < 50) {// if the obstacle distance is too close, reverse the travelling direction
    if (barDegree < 90)       // choose to reverse direction according to the angle with obstacle
      ctrlCar(135, FORWARD, spd);   // control steering and moving on
    else
      ctrlCar(45, FORWARD, spd);    // control steering and moving on
    delay(1000);             // time to move on
  }
  else {                     // if the obstacle distance is not close, move on
    ctrlCar(90, FORWARD, spd);      // control the smart car move on
    delay(1000);             // time to move on
  }
  ctrlCar(90, FORWARD, 0);   // make the smart car stop for preparation of next scanning
}

void ctrlCar(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);
  digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
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

  if (pingTime != 0) {  // if the measure is not overtime
    distance = pingTime * soundVelocity / 2 / 10000;  // calculate the obstacle distance(cm) according to the time of high level returned
    return distance;    // return distance(cm)
  }
  else                  // if the measure is overtime
    return maxDistance; // returns the maximum distance(cm)
}

