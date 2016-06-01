/*
  Remote control

  The sample code of Freenove Smart Car Shield for Arduino UNO, can make the smart car 
  work in remote control mode.
  This code applies to Freenove Three-wheeled Smart Car. You need to install NFR24L01 for the 
  expansion shield. The remote control use Freenove Smart Car Remote Shield for Arduino UNO.

  created 2016/5/20
  by http://www.freenove.com
*/

#include <SPI.h>
#include "RF24.h"
RF24 radio(9, 10);            // define the object to control NRF24L01
byte addresses[6] = "Free1";  // define communication address which should correspond to remote control
int data[7];                  // define array used to save the communication data

#include <Servo.h>
Servo dirServo;          // define servo to control turning of smart car
int dirServoPin = 2;      // define pin for signal line of the last servo
float dirServoOffset = 6; // define a variable for deviation(degree) of the servo

int dirAPin = 7,    // define pin used to control rotational direction of motor A
    pwmAPin = 6,    // define pin for PWM used to control rotational speed of motor A
    dirBPin = 4,    // define pin used to control rotational direction of motor B
    pwmBPin = 5,    // define pin for PWM used to control rotational speed of motor B
    snsAPin = A0,   // define pin for detecting current of motor A
    snsBPin = A1,   // define pin for detecting current of motor B
    buzzerPin = 3,  // define pin for buzzer
    ledRPin = A3,   // define R pin of RGBLED
    ledGPin = A4,   // define G pin of RGBLED
    ledBPin = A5;   // define B pin of RGBLED

#define FORWARD LOW
#define BACKWARD HIGH

void setup() {
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openReadingPipe(1, addresses);// open delivery channel
  radio.startListening();             // start monitoring

  Serial.begin(9600); // initialize serial port

  dirServo.attach(dirServoPin);  // attaches the servo on servoDirPin to the servo object

  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode
  pinMode(buzzerPin, OUTPUT); // set buzzerPin to output mode
  pinMode(ledRPin, OUTPUT);   // set ledRPin to output mode
  pinMode(ledGPin, OUTPUT);   // set ledGPin to output mode
  pinMode(ledBPin, OUTPUT);   // set ledBPin to output mode

  digitalWrite(ledRPin, HIGH);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledBPin, HIGH);
}

void loop()
{
  if ( radio.available()) {             // if receive the data
    while (radio.available()) {         // read all the data
      radio.read( data, sizeof(data) ); // read data
    }

    // calculate the steering angle of servo according to the direction joystick of remote control and the deviation
    int dirServoDegree = map(data[0], 0, 1023, 135, 45) - (data[5] - 512) / 25;
    // get the steering angle and speed of servo according to the speed joystick of remote control and the deviation
    int motorSpd = data[1] - 512 + (data[6] - 512) / 10;
    bool motorDir = motorSpd > 0 ? BACKWARD : FORWARD;
    motorSpd = abs(constrain(motorSpd, -512, 512));
    motorSpd = map(motorSpd, 0, 512, 0, 255);
    // control the steering and travelling of the smart car
    ctrlCar(dirServoDegree, motorDir, motorSpd);

    // control the buzzer
    if (!data[2])
      tone(buzzerPin, 2000);
    else
      noTone(buzzerPin);

    // control the RGB LED
    digitalWrite(ledRPin, data[3]);
    digitalWrite(ledBPin, data[4]);
  }

  // send motor current to serial port
  float iMotorA = analogRead(snsAPin) * 5.0 / 1024 / 30 / 0.05;
  float iMotorB = analogRead(snsBPin) * 5.0 / 1024 / 30 / 0.05;
  Serial.print("iMotorA: ");
  Serial.print(iMotorA);
  Serial.print(" A ,iMotorB: ");
  Serial.print(iMotorB);
  Serial.println(" A");
}

void ctrlCar(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);
  digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
}
