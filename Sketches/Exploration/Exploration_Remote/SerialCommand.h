/*
  Class     SerialCommand
  Author    Ethan Pan @ Freenove (http://www.freenove.com)
  Date      2016/8/13
  Brief     This class is used to save serial command.
  Copyright Copyright © Freenove (http://www.freenove.com)
  License   Creative Commons Attribution ShareAlike 3.0
            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
*/

class serialCommand
{
  public:
    // Trans control command, range 128 ~ 255
    const byte transStart = 128;
    const byte transEnd = 129;

    // General command , range 0 ~ 127
    // The odd command is sent by the requesting party
    // The even command is sent by the responding party
    // Request echo, to confirm the device
    const byte requestEcho = 0;      // Comm
    // Respond echo, to tell this is the device
    const byte echo = 1;             // Comm
    // Request 1 analog value
    const byte requestCarInfo = 21;  // Comm
    // Respond 1 analog value
    // Comm scanDistance[ptScanTimes] iMotorA iMotorB signalStrength
    const byte carInfo = 22;
} SerialCommand;

