/*
  Sketch    Exploration mode sketch for remote
  Platform  Freenove Three-wheeled Smart Car Kit for Arduino
  Author    Ethan Pan @ Freenove (http://www.freenove.com)
  Date      2016/10/7
  Brief     This sketch is used to achieve exploration mode for Freenove Three-wheeled
            Smart Car Kit for Arduino. This sketch needs to be uploaded to the remote.
            The remote needs to install NFR24L01 wireless communication module.
  Copyright Copyright © Freenove (http://www.freenove.com)
  License   Creative Commons Attribution ShareAlike 3.0
            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
*/

#include "SerialCommand.h"
// NRF24L01
#include <SPI.h>
#include "RF24.h"
RF24 radio(9, 10);                // define the object to control NRF24L01
const byte addresses[6] = "Free1";// define communication address which should correspond to remote control
// pin
const int pot1Pin = A0,           // define POT1
          pot2Pin = A1,           // define POT2
          joystickXPin = A2,      // define pin for direction X of joystick
          joystickYPin = A3,      // define pin for direction Y of joystick
          led1Pin = 6,            // define pin for LED1 which is close to POT1 and used to indicate the state of POT1
          led2Pin = 5,            // define pin for LED2 which is close to POT2 and used to indicate the state of POT2
          led3Pin = 8;            // define pin for LED3 which is close to NRF24L01 and used to indicate the state of NRF24L01
// scan distance of obstacle
const int ptScanTimes = 11;       // define the number of scan times per cycle
// calculate signal strength
const byte signalSamples = 10;
byte signalStrength;
// wireless communication
int dataRead[ptScanTimes + 2];    // define array used to save the read data
int dataWrite[4];                 // define array used to save the write data

void setup() {
  // serial port
  Serial.begin(115200);
  // NRF24L01
  radio.begin();                      // initialize RF24
  radio.setPALevel(RF24_PA_LOW);      // set power amplifier (PA) level
  radio.setDataRate(RF24_1MBPS);      // set data rate through the air
  radio.setRetries(0, 15);            // set the number and delay of retries
  radio.openWritingPipe(addresses);   // open a pipe for writing
  radio.openReadingPipe(1, addresses);// open a pipe for reading
  radio.stopListening();              // stop listening for incoming messages
  // led
  pinMode(led1Pin, OUTPUT);     // set led1Pin to output mode
  pinMode(led2Pin, OUTPUT);     // set led2Pin to output mode
  pinMode(led3Pin, OUTPUT);     // set led3Pin to output mode
}

void loop()
{
  // put the values of rocker, switch and potentiometer into the array
  dataWrite[0] = analogRead(pot1Pin);
  dataWrite[1] = analogRead(pot2Pin);
  dataWrite[2] = analogRead(joystickXPin);
  dataWrite[3] = analogRead(joystickYPin);

  // write radio data
  if (radio.write(dataWrite, sizeof(dataWrite)))
  {
    radio.startListening();             // start monitoring
    digitalWrite(led3Pin, HIGH);
    delay(20);
    digitalWrite(led3Pin, LOW);

    // read radio data
    if ( radio.available()) {           // if receive the data
      while (radio.available())         // read all the data
        radio.read(dataRead, sizeof(dataRead)); // read data
      calcSignal(true);
    }
    else {
      calcSignal(false);
    }
    radio.stopListening();              // stop monitoring
  }
  else {
    calcSignal(false);
  }

  // make LED emit different brightness of light according to analog of potentiometer
  analogWrite(led1Pin, map(analogRead(pot1Pin), 0, 1023, 0, 255));
  analogWrite(led2Pin, map(analogRead(pot2Pin), 0, 1023, 0, 255));
}

void calcSignal(boolean state) {
  static boolean states[signalSamples];
  for (int i = 0; i < signalSamples - 1; i++) {
    states[i] = states[i + 1];
  }
  states[signalSamples - 1] = state;

  int comSucceed = 0;
  for (int i = 0; i < signalSamples; i++) {
    if (states[i])
      comSucceed++;
  }

  signalStrength = comSucceed * 100 / signalSamples;
}

void serialEvent() {
  static byte inData[64];
  static byte inDataNum = 0;

  while (Serial.available() > 0)
  {
    byte inByte = Serial.read();
    if (inByte == SerialCommand.transStart)
      inDataNum = 0;
    inData[inDataNum++] = inByte;
    if (inByte == SerialCommand.transEnd)
      if (inData[0] == SerialCommand.transStart)
        break;
  }

  if (inData[0] == SerialCommand.transStart && inData[inDataNum - 1] == SerialCommand.transEnd)
  {
    Serial.write(SerialCommand.transStart);
    if (inData[1] == SerialCommand.requestEcho)
    {
      Serial.write(SerialCommand.echo);
    }
    else if (inData[1] == SerialCommand.requestCarInfo)
    {
      Serial.write(SerialCommand.carInfo);
      for (int i = 0; i < sizeof(dataRead) / sizeof(int); i++)
      {
        Serial.write(dataRead[i] / 128);
        Serial.write(dataRead[i] % 128);
      }
      Serial.write(signalStrength / 128);
      Serial.write(signalStrength % 128);
    }
    Serial.write(SerialCommand.transEnd);
  }
}

