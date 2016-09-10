/*
  Sketch    Remote control mode sketch for car
  Platform  Freenove Three-wheeled Smart Car Kit for Arduino
  Author    Ethan Pan @ Freenove (http://www.freenove.com)
  Date      2016/8/13
  Brief     This sketch is used to achieve remote control mode for Freenove Three-wheeled
            Smart Car Kit for Arduino. This sketch needs to be uploaded to the car.
            The car needs to install NFR24L01 wireless communication module, can choose to
            install RGB LED module and passive buzzer module.
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
byte dirServoOffset = 0;          // define a variable for deviation(degree) of the servo
// motor
const int dirAPin = 7,            // define pin used to control rotational direction of motor A
          pwmAPin = 6,            // define pin for PWM used to control rotational speed of motor A
          dirBPin = 4,            // define pin used to control rotational direction of motor B
          pwmBPin = 5,            // define pin for PWM used to control rotational speed of motor B
          snsAPin = A0,           // define pin for detecting current of motor A
          snsBPin = A1,           // define pin for detecting current of motor B
          buzzerPin = 3,          // define pin for buzzer
          ledRPin = A3,           // define R pin of RGBLED
          ledGPin = A4,           // define G pin of RGBLED
          ledBPin = A5;           // define B pin of RGBLED
const byte motorSpeed = 255;      // define motor speed(0-255)
#define FORWARD LOW
#define BACKWARD HIGH
// wireless communication
int dataRead[8];                  // define array used to save the read data

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
  // motor
  pinMode(dirAPin, OUTPUT);     // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);     // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);     // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);     // set pwmBPin to output mode
  // pin
  pinMode(buzzerPin, OUTPUT);   // set buzzerPin to output mode
  pinMode(ledRPin, OUTPUT);     // set ledRPin to output mode
  pinMode(ledGPin, OUTPUT);     // set ledGPin to output mode
  pinMode(ledBPin, OUTPUT);     // set ledBPin to output mode
  digitalWrite(ledRPin, HIGH);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledBPin, HIGH);
}

void loop()
{
  // read radio data
  if ( radio.available()) {             // if receive the data
    while (radio.available()) {         // read all the data
      radio.read(dataRead, sizeof(dataRead));   // read data
    }

    // calculate the angle of turning
    int dirServoDegree = map(dataRead[2], 0, 1023, 135, 45) - (dataRead[0] - 512) / 25;
    // calculate speed of moving
    int motorSpd = dataRead[3] - 512 + (dataRead[1] - 512) / 10;
    bool motorDir = motorSpd > 0 ? BACKWARD : FORWARD;
    motorSpd = abs(constrain(motorSpd, -512, 512));
    motorSpd = map(motorSpd, 0, 512, 0, motorSpeed);

    // control the turning and moving of the car
    ctrlCar(dirServoDegree, motorDir, motorSpd);

    // control the buzzer
    if (!dataRead[4])
      tone(buzzerPin, 2000);
    else
      noTone(buzzerPin);

    // control the RGB LED
    digitalWrite(ledRPin, dataRead[5]);
    digitalWrite(ledGPin, dataRead[6]);
    digitalWrite(ledBPin, dataRead[7]);
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

