/*
  ===============================================
  Example sketch for CurieIMU library for Intel(R) Curie(TM) devices.
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
  class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

  ===============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================

  Genuino 101 CurieIMU Orientation Visualiser
  Hardware Required:
    Arduino/Genuino 101

  Modified Nov 2015
  by Helena Bisby <support@arduino.cc>
  This example code is in the public domain
  http://arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
*/

#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>

Madgwick filter; // initialise Madgwick object
int ax, ay, az;
int gx, gy, gz;
float yaw;
float pitch;
float roll;
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

#define sclk 2
#define mosi 3
#define dc   4
#define cs   5
#define rst  6

Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);
String names[] = {"y", "p", "r"};
void setup() {
  // initialize Serial communication
  Serial.begin(9600);

  // initialize device
  CurieIMU.begin();

  if (calibrateOffsets == 1) {
    // use the code below to calibrate accel/gyro offset values
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Z_AXIS)); Serial.print("\t");
    Serial.println("");

    // To manually configure offset compensation values, use the following methods instead of the autoCalibrate...() methods below
    //    CurieIMU.setGyroOffset(X_AXIS, 220);
    //    CurieIMU.setGyroOffset(Y_AXIS, 76);
    //    CurieIMU.setGyroOffset(Z_AXIS, -85);
    //    CurieIMU.setAccelerometerOffset(X_AXIS, -76);
    //    CurieIMU.setAccelerometerOffset(Y_AXIS, -235);
    //    CurieIMU.setAccelerometerOffset(Z_AXIS, 168);

    //IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!

    Serial.print("Starting Gyroscope calibration...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");
    Serial.print("Starting Acceleration calibration...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
    Serial.println("");
  }
  tft.begin();
  tft.fillScreen(0x0000);
}
void loop() {
  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

  // use function from MagdwickAHRS.h to return quaternions
  filter.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);

  // functions to find yaw roll and pitch from quaternions
  float mapYaw = mapVal(filter.getYaw(), -1, 1, 10, 76);
  float mapPitch = mapVal(filter.getPitch(), -1, 1, 10, 76);
  float mapRoll = mapVal(filter.getRoll(), -1, 1, 10, 76);


  tft.setCursor(0, 0);
  tft.fillRect(24, 10, 11, 66, 0x0000);
  tft.fillRect(54, 10, 11, 66, 0x0000);
  tft.fillRect(84, 10, 11, 66, 0x0000);
  for (int i = 1; i <= 3; i += 1) {
    tft.drawLine(i * 30, 10, i * 30, tft.height() - 20, 0xFFFF);
    tft.setCursor(i * 30, tft.height() - 10);
    tft.print(names[i - 1]);
  }
  tft.drawRect(25, mapYaw, 10, 5, 0xFFFF);
  tft.drawRect(55, mapPitch, 10, 5, 0xFFFF);
  tft.drawRect(85, mapRoll, 10, 5, 0xFFFF);

}
//this function works better in this case than the arduino function
float mapVal(float s, float a1, float a2, float b1, float b2)
{
  return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}
