/* //<>//
 ******************************************************************************
 * Sketch    Exploration mode sketch for Processing
 * Platform  Freenove Three-wheeled Smart Car Kit for Arduino
 * Author    Ethan Pan @ Freenove (http://www.freenove.com)
 * Date      2016/8/14
 ******************************************************************************
 * Brief
 *   This sketch is used to achieve exploration mode for Freenove Three-wheeled 
 *   Smart Car Kit for Arduino. This sketch needs to be run with Processing. 
 *   The remote needs to be connected to the computer running this sketch. 
 ******************************************************************************
 * Copyright
 *   Copyright © Freenove (http://www.freenove.com)
 * License
 *   Creative Commons Attribution ShareAlike 3.0 
 *   (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// scan distance of obstacle
final int ptScanInterval = 10;      // define the interval(degree) of each scan
final int ptScanTimes = 11;         // define the number of scan times per cycle
/* Private variables ---------------------------------------------------------*/
SerialDevice serialDevice = new SerialDevice(this);
// serial communication
int dataRead[] = new int[ptScanTimes + 3];  // serial data read

void setup()
{
  size(960, 540);
  background(102);
  textAlign(CENTER, CENTER);
  textSize(64);
  text("Starting...", width / 2, (height - 40) / 2);
  textSize(16);
  text("www.freenove.com", width / 2, height - 20);
  frameRate(1000 / 40);
}

void draw()
{
  if (!serialDevice.active())
  {
    if (!serialDevice.start())
    {
      delay(1000);
      return;
    }
  }

  dataRead = serialDevice.requestCarInfo();
  if (dataRead != null)
  {
    background(102);
    showGUI();
  }
}

void showGUI()
{
  stroke(64, 64, 64);
  noFill();
  // circle
  for (int i = 0; i < height; i += 60)
  {
    ellipse(width / 2, height / 2, i, i);
  }
  // line
  for (int i = 0; i < 360; i += 45)
  {
    float dx = (height / 2 - 30) * sin(i * PI / 180);
    float dy = (height / 2 - 30) * cos(i * PI / 180);
    line(width / 2, height / 2, width / 2 + dx, height / 2 + dy);
  }

  fill(255, 255, 255);
  textAlign(CENTER, CENTER);
  textSize(16);
  // circle text
  text("0CM", width / 2, height / 2 + 12);
  for (int i = 60; i < height - 60; i += 60)
  {
    text(i / 6, width / 2 + i / 2 + 15, height / 2 + 12);
    text(i / 6, width / 2 - i / 2 - 15, height / 2 + 12);
  }
  // line text
  for (int i = 45; i < 360; i += 45)
  {
    float dx = (height / 2 - 12) * sin(i * PI / 180);
    float dy = (height / 2 - 12) * cos(i * PI / 180);
    text((i - 180) + "°", width / 2 + dx, height / 2 + dy);
  }
  float dx = (height / 2 - 12) * sin(0 * PI / 180);
  float dy = (height / 2 - 12) * cos(0 * PI / 180);
  text("±" + 180 + "°", width / 2 + dx, height / 2 + dy);    

  // point
  fill(0, 255, 0);
  for (int i = 0; i < ptScanTimes; i ++)
  {
    float x = dataRead[i] * sin(ptScanInterval * (i - (ptScanTimes - 1) / 2) * PI / 180); 
    x *= -3;
    float y = dataRead[i] * cos(ptScanInterval * (i - (ptScanTimes - 1) / 2) * PI / 180); 
    y *= 3;
    if (dataRead[i] <= 80)
      ellipse(width / 2 + x, height / 2 - y, 10, 10);
  }
  // scan distance
  fill(255, 255, 255);
  textAlign(LEFT);
  textSize(16);
  text("Distance", 10, 16);
  for (int i = 0; i < ptScanTimes; i++)
  {
    text("[" + i + "]" + " ", 10, 32 + 16 * i);
    if (dataRead[i] > 199)
      text("--", 48, 32 + 16 * i);
    else
      text(dataRead[i] + "cm", 48, 32 + 16 * i);
  }
  // motor current
  int iMotorA = dataRead[ptScanTimes];
  int iMotorB = dataRead[ptScanTimes + 1];
  textAlign(LEFT, CENTER);
  text("Current", 10, height - 20 - 16 * 2);
  text("Motor A: " + iMotorA / 100 + "." + iMotorA / 10 % 10 + iMotorA % 10 + "A", 10, height - 20 - 16 * 1);
  text("Motor B: " + iMotorB / 100 + "." + iMotorB / 10 % 10 + iMotorB % 10 + "A", 10, height - 20 - 16 * 0);
  // signal strength
  int signalStrength = dataRead[ptScanTimes + 2];
  textAlign(RIGHT);
  text("Signal", width - 10, 16);
  text(signalStrength + "%", width - 10, 32);
  if (signalStrength == 0)
    showInfo("Smatr car offline");

  // link
  fill(255, 255, 255);
  textAlign(RIGHT, CENTER);
  textSize(16);
  text("Press Enter to visit www.freenove.com", width - 10, height - 20);
}

void showInfo(String info)
{
  rectMode(CENTER);
  stroke(255, 0, 0);
  fill(102);
  rect(width / 2, height / 2, width / 3, height / 4);
  fill(255, 255, 255);
  textAlign(CENTER, CENTER);
  textSize(24);
  text("Warning", width / 2, height / 2 - 36);
  textSize(36);
  text(info, width / 2, height / 2 + 24);
}

void keyPressed() 
{
  if (key == '\n' || key == '\r')
  {
    link("http://www.freenove.com");
  }
}