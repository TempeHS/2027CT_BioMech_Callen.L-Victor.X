#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// Intialize Sensors
LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool imuReady = false;

// Active Sensor Values
float aX = 0.0f;
float aY = 0.0f;
float aZ = 0.0f;

float gX = 0.0f;
float gY = 0.0f;
float gZ = 0.0f;


void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    // Successful Connection
    Serial.println("Device Successfully Linked");
    delay(2000);

    Serial.println("Stay Stationary, Gyroscope Calibration is Starting Soon...");
    delay(2000);

    Serial.println("3");
    delay(1000);

    Serial.println("2");
    delay(1000);

    Serial.println("1");
    delay(1000);

    // Calibrating Gyroscope
    calibrateGyroscope();

    Serial.println("Finished Gyroscope Calibration!");
    delay(1000);

    Serial.print("Gyro Offset X: ");
    Serial.println(gyroOffsetX, 4);
    Serial.print("Gyro Offset Y: ");
    Serial.println(gyroOffsetY, 4);
    Serial.print("Gyro Offset Z: ");
    Serial.println(gyroOffsetZ, 4);

    delay(2500);

    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void loop() {
  // Calculates for Significant Changes
  while (samplesRead < numSamples) {

    // STEP 0: Print Run Number AND bars for readability
    Serial.println("==================================");
    Serial.print("Run Number: ");
    Serial.println(runNumber);

    // STEP 1: Begin Data Pruning
    delay(pruneReadingDiff);

    // STEP 2: Collect the new accelerometer values
    float nextAX = myIMU.readFloatAccelX();
    float nextAY = myIMU.readFloatAccelY();
    float nextAZ = myIMU.readFloatAccelZ();

    // STEP 3: Calculates the accelerometer change in data
    float dAX = nextAX - aX;
    float dAY = nextAY - aY;
    float dAZ = nextAZ - aZ;

    // STEP 4: Accelerometer Thresholds and Further Prune
    float filteredAX = (fabsf(dAX) > accelerationMargin) // Accel X (new value, diff. ab. value., if margin > store, else 0.)
                   ? dAX : 0.0f;

    float filteredAY = (fabsf(dAY) > accelerationMargin)
                      ? dAY : 0.0f;

    float filteredAZ = (fabsf(dAZ) > accelerationMargin)
                      ? dAZ : 0.0f;

    // STEP 5: Reads Gyro and Further Prunes
    gX = myIMU.readFloatGyroX() - gyroOffsetX;
    gY = myIMU.readFloatGyroY() - gyroOffsetY;
    gZ = myIMU.readFloatGyroZ() - gyroOffsetZ;

    float filteredGX = (fabsf(gX) > gyroscopeMargin) // Gyro X (new value, diff. ab. value., if margin > store, else 0.)
                      ? gX : 0.0f;

    float filteredGY = (fabsf(gY) > gyroscopeMargin)
                      ? gY : 0.0f;

    float filteredGZ = (fabsf(gZ) > gyroscopeMargin)
                      ? gZ : 0.0f;

    // STEP 6: Create the movement flags for Accel Movements
    accelMovementX = filteredAX != 0.0f; // Make filteredAX TRUE if its anything but 0
    accelMovementY = filteredAY != 0.0f;
    accelMovementZ = filteredAZ != 0.0f;

    gyroMovementX = filteredGX != 0.0f;
    gyroMovementY = filteredGY != 0.0f;
    gyroMovementZ = filteredGZ != 0.0f;

    // STEP 7: Make Stored Accelerometer Data the new reference
    aX = nextAX;
    aY = nextAY;
    aZ = nextAZ;

    // STEP 8: Create the movement array
    int8_t directions[6] = {
      getDirection(dAX, accelerationMargin),
      getDirection(dAY, accelerationMargin),
      getDirection(dAZ, accelerationMargin),

      getDirection(gX, gyroscopeMargin),
      getDirection(gY, gyroscopeMargin),
      getDirection(gZ, gyroscopeMargin)
    };

    // STEP 9: Print Values on a Table Design
    Serial.println("Movement sample:");

    Serial.print("Accel: ");
    Serial.print(directions[0]);
    Serial.print(", ");
    Serial.print(directions[1]);
    Serial.print(", ");
    Serial.println(directions[2]);

    Serial.print("Gyro:  ");
    Serial.print(directions[3]);
    Serial.print(", ");
    Serial.print(directions[4]);
    Serial.print(", ");
    Serial.println(directions[5]);

    Serial.println();

    // Add Samples Read to keep program in sync
    samplesRead++;
    runNumber++; // Counter :D
    Serial.println("==================================");
  }
}

