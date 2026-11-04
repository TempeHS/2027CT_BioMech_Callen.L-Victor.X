#include "LSM6DS3.h"
#include <Wire.h>

bool powerOnState = 1;                            // MARK: [TEMPORARY!] Power On State is True

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);                    // Address at 0x6a 
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 2.5;          // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;

void setup() {
  Serial.begin(115200);                           // Note: Align with Serial Terminal Number
 
  while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void loop() {
  while (powerOnState = True) {
    // wait for significant motion

    unsigned int runNumber = 1;

    while (samplesRead == numSamples) {
      // read the acceleration data
      aX = myIMU.readFloatAccelX();
      aY = myIMU.readFloatAccelY();
      aZ = myIMU.readFloatAccelZ();

      // sum up the absolutes
      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

      // check if it's above the threshold
      if (aSum >= accelerationThreshold) {
        // reset the sample read count
        samplesRead = 0;
        break;
      }
    }

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
}
