/*
 *******************************************************************************
 * Class   SerialDevice
 * Author  Ethan Pan @ Freenove (http://www.freenove.com)
 * Date    2016/8/14
 *******************************************************************************
 * Brief
 *   This class is used to connect a specific serial port.
 *   It will automatically detect and connect to a device (serial port) which 
 *   use the same trans format.
 *******************************************************************************
 * Serial data formats
 *   Baud    115200
 *   Data    Range 0 ~ 127 per data byte
 *   Format  0          1       2       ... n-1       n 
 *           transStart data[0] data[1] ... data[n-1] transEnd
 *******************************************************************************
 * Copyright
 *   Copyright © Freenove (http://www.freenove.com)
 * License
 *   Creative Commons Attribution ShareAlike 3.0 
 *   (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
import processing.serial.*;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * Brief  This class is used to save serial command
 *****************************************************************************/
class SerialCommand
{
  // Trans control command, range 128 ~ 255
  final static byte transStart = (byte)128;
  final static byte transEnd = (byte)129;

  // General command , range 0 ~ 127
  // The odd command is sent by the requesting party
  // The even command is sent by the responding party
  // Request echo, to confirm the device
  final static byte requestEcho = 0;      // Comm
  // Respond echo, to tell this is the device
  final static byte echo = 1;             // Comm
  // Request 1 analog value
  final static byte requestCarInfo = 21;  // Comm
  // Respond 1 analog value
  // Comm scanDistance[ptScanTimes] iMotorA iMotorB signalStrength
  final static byte carInfo = 22;
}

/*
 * Brief  This class is used to automatically detect and connect to a device 
 *        (serial port) which use the same trans format.
 *****************************************************************************/
class SerialDevice
{
  private final static int readTimeOut = 30;

  private PApplet parent;
  private boolean active = false;
  public Serial serial;
  public String serialName;

  SerialDevice(PApplet pApplet)
  {
    parent = pApplet;
  }

  public boolean active()
  {
    return active;
  }

  public boolean start()
  {
    stop();
    println(time() + "Start connect device...");
    String[] serialNames = Serial.list();
    if (serialNames.length == 0)
    {
      println(time() + "No serial port detected, waiting for connection...");
      return false;
    }
    print(time() + "Detected serial port: ");
    for (int i = 0; i < serialNames.length; i++)
      print(serialNames[i] + " ");
    println("");
    for (int i = 0; i < serialNames.length; i++)
    {
      println(time() + "Attempt to connect " + serialNames[i] + "...");
      try {
        serial = new Serial(parent, serialNames[i], 115200);
        serial.clear();
        delay(2000);
        byte[] data = new byte[1];
        data[0] = SerialCommand.requestEcho;
        write(serial, data);
        delay(200);
        data = read(serial);
        if (data != null)
        {
          if (data[0] == SerialCommand.echo)
          {
            serialName = serialNames[i];
            println(time() + "Device connection success: " + serialDevice.serialName);
            active = true;
            return true;
          }
        }
        serial.stop();
      }
      catch (Exception e) {
        e.printStackTrace();
      }
    }
    println(time() + "Device connection failed");
    return false;
  }

  public void stop()
  {
    if (active())
    {
      active = false;
      serial.stop();
    }
  }

  public boolean write(byte[] data)
  {
    if (active())
    {
      write(serial, data);
      return true;
    }
    println(time() + "Write failed");
    return false;
  }

  public byte[] read()
  {
    if (active())
    {
      byte[] data = read(serial);
      if (data != null)
        return data;
    }
    println(time() + "Read failed");
    return null;
  }

  private void write(Serial serial, byte[] data)
  {
    serial.write(SerialCommand.transStart);
    serial.write(data);
    serial.write(SerialCommand.transEnd);
  }

  private byte[] read(Serial serial)
  {
    byte[] inData = new byte[64];
    int inDataNum = 0;
    int startTime = millis();

    do
    {
      if (serial.available() > 0)
      {
        byte[] inTemp = new byte[1];
        serial.readBytes(inTemp);
        byte inByte = inTemp[0];

        if (inByte == SerialCommand.transStart)
          inDataNum = 0;
        inData[inDataNum++] = inByte;
        if (inByte == SerialCommand.transEnd)
          if (inData[0] == SerialCommand.transStart)
            break;
        startTime = millis();
      }
    } 
    while (millis () - startTime < readTimeOut);

    if (inData[0] == SerialCommand.transStart && inData[inDataNum - 1] == SerialCommand.transEnd)
    {
      byte[] data = new byte[inDataNum - 2];
      for (int i = 0; i < inDataNum - 2; i++)
        data[i] = inData[i + 1];
      return data;
    }

    return null;
  }

  public int[] requestCarInfo()
  {
    byte[] data = new byte[1];
    data[0] = SerialCommand.requestCarInfo;
    write(data);
    data = read();
    if (data != null)
    {
      if (data[0] == SerialCommand.carInfo)
      {
        int[] carInfo = new int[(data.length - 1) / 2];
        for (int i = 0; i < carInfo.length; i++)
        {
          carInfo[i] = data[i * 2 + 1] * 128 + data[i * 2 + 2];
        }
        return carInfo;
      }
    }
    return null;
  }

  public String time()
  {
    return hour() + ":" + minute() + ":" + second() + " ";
  }
}

