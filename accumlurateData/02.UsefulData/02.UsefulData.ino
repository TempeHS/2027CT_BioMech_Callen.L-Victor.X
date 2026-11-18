#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

float aX, aY, aZ, gX, gY, gZ;

// Accelerometer Min Thresholds
const float accelerationThreshold = 2.5; // MIN. Combined Acceleration required to trigger recording
const int numSamples = 119;              // Initialisation of 119 as sample
int samplesRead = numSamples;            // Sample Read matching numSample above.

// Sensors Margin of Change
constexpr float accelerationMargin = 0.15f;
constexpr float gyroscopeMargin = 0.15f;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device Successfully Linked");
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void loop() {

  // Accelerometer Min Thresholds: Waits for significant motion
  while (samplesRead == numSamples) {
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    float aSum = fabs(aX) + fabs(aY) + fabs(aZ); // Absolute value derived from: aSum = |aX| + |aY| + |aZ|

    if (aSum >= accelerationThreshold) { // When aSum >= accelerationThreshold, continues and resets sampleRead (=numSample till)
      samplesRead = 0;
      break;
    }
  }

  unsigned int runNumber = 1;

  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
    // check if both new acceleration and gyroscope data is
    // available
    // read the acceleration and gyroscope data

    samplesRead++;

    // print the data in CSV format
    Serial.print("Run Number: ");
    Serial.println(runNumber);

    Serial.println();
    Serial.println();
    Serial.println("Accelerometer Values");

    Serial.print(myIMU.readFloatAccelX(), 3);
    Serial.print(',');

    Serial.print(myIMU.readFloatAccelY(), 3);
    Serial.print(',');

    Serial.print(myIMU.readFloatAccelZ(), 3);
    Serial.print(',');


    Serial.println();
    Serial.println();

    Serial.println("Gryo Values");

    Serial.print(myIMU.readFloatGyroX(), 3);
    Serial.print(',');

    Serial.print(myIMU.readFloatGyroY(), 3);
    Serial.print(',');

    Serial.print(myIMU.readFloatGyroZ(), 3);
    Serial.println();
    Serial.println();

    runNumber++;

    delay(5000);

    if (samplesRead == numSamples) {
      // add an empty line if it's the last sample
      Serial.println();
    }
  }
}

