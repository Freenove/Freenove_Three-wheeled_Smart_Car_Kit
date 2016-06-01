/*
  Remote

  The sample code of Freenove Smart Car Remote Shield for Arduino UNO, can make the smart car 
  work in remote control mode.
  This code applies to Freenove Three-wheeled Smart Car, using Freenove Smart Car Shield.
  Shield for Arduino UNO needs to install the NFR24L01 module.

  created 2016/5/20
  by http://www.freenove.com
*/

#include <SPI.h>
#include "RF24.h"
RF24 radio(9, 10);            // define the object to control NRF24L01
byte addresses[6] = "Free1";  // define communication address which should correspond to remote control
int data[7];                  // define array used to save the communication data

int pot1Pin = A0,       // define POT1
    pot2Pin = A1,       // define POT2
    led1Pin = 6,        // define pin for LED1 which is close to POT1 and used to indicate the state of POT1
    led2Pin = 5,        // define pin for LED2 which is close to POT2 and used to indicate the state of POT2
    led3Pin = 8,        // define pin for LED3 which is close to NRF24L01 and used to indicate the state of NRF24L01
    s1Pin = 4,          // define pin for S1
    s2Pin = 3,          // define pin for S2
    s3Pin = 2,          // define pin for S3
    joystickXPin = A2,  // define pin for direction X of joystick
    joystickYPin = A3;  // define pin for direction Y of joystick

void setup() {
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openWritingPipe(addresses);   // open delivery channel
  radio.stopListening();              // stop monitoring

  pinMode(led1Pin, OUTPUT);   // set led1Pin to output mode
  pinMode(led2Pin, OUTPUT);   // set led2Pin to output mode
  pinMode(led3Pin, OUTPUT);   // set led3Pin to output mode
}

void loop()
{
  // put the values of rocker, switch and potentiometer into the array
  data[0] = analogRead(joystickXPin);
  data[1] = analogRead(joystickYPin);
  data[2] = digitalRead(s1Pin);
  data[3] = digitalRead(s2Pin);
  data[4] = digitalRead(s3Pin);
  data[5] = analogRead(pot1Pin);
  data[6] = analogRead(pot2Pin);

  // send array data. If the sending succeeds, open signal LED
  if (radio.write( data, sizeof(data) ))
    digitalWrite(led3Pin, HIGH);

  // make LED emit different brightness of light according to analog of potentiometer
  analogWrite(led1Pin, map(data[5], 0, 1023, 0, 255));
  analogWrite(led2Pin, map(data[6], 0, 1023, 0, 255));

  // delay for a period of time, then turn off the signal LED for next sending
  delay(2);
  digitalWrite(led3Pin, LOW);
}
