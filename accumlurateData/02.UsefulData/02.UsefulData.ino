#include "LSM6DS3.h"
#include <Wire.h>

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 2.5; // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;

//Sensors Margin of Change
const unsigned float accelerationMargin = 0.15;
const unsigned float gyroscopeMargin = 1;
//Data is based upon testing of sensors from 01

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void loop() {

}
