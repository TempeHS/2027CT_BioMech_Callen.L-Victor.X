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

// Key Values
bool accelMovementX = 0;
bool gyroMovementX = 0;

bool accelMovementY = 0;
bool gyroMovementY = 0;

bool accelMovementZ = 0;
bool gyroMovementZ = 0;

// Data Merger Component for Types of Movements
enum Movement {
    STATIONARY,
    FORWARD_LEFT_UP,
    FORWARD_LEFT_STRAIGHT,
    FORWARD_DOWN_STRAIGHT,
    BACK_DOWN_DOWN,
    UNKNOWN,
};

Movement currentMovement = UNKNOWN;

// Delay for Next Reading prior to Pruning (in ms)
unsigned int pruneReadingDiff = 200;

// Run no. counter
unsigned int runNumber = 1;

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

  // Calculates for Significant Changes
  while (samplesRead < numSamples) {

    // STEP1: Original Sensor Readings Values.
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    gX = myIMU.readFloatGyroX();
    gY = myIMU.readFloatGyroY();
    gZ = myIMU.readFloatGyroZ();

    samplesRead++; // Update run no. number

    // STEP2: A Short Delay prior to next readings
    delay(pruneReadingDiff);

    // STEP3: Collect next values and filters out things below margin
    float nextValue;

    nextValue = myIMU.readFloatAccelX(); // Accel X (new value, diff. ab. value., if margin > store, else 0.)
    aX = (fabsf(nextValue - aX) > accelerationMargin)
         ? nextValue : 0.0f; accelMovementX = 1;

    nextValue = myIMU.readFloatAccelY(); // Accel Y
    aY = (fabsf(nextValue - aY) > accelerationMargin)
         ? nextValue : 0.0f; accelMovementY = 1;

    nextValue = myIMU.readFloatAccelZ(); // Accel Z
    aZ = (fabsf(nextValue - aZ) > accelerationMargin)
         ? nextValue : 0.0f; accelMovementZ = 1;

    nextValue = myIMU.readFloatGyroX(); // Gyro X
    gX = (fabsf(nextValue - gX) > gyroscopeMargin)
         ? nextValue : 0.0f; gyroMovementX = 1;

    nextValue = myIMU.readFloatGyroY(); // Gyro Y
    gY = (fabsf(nextValue - gY) > gyroscopeMargin)
         ? nextValue : 0.0f; gyroMovementY = 1;

    nextValue = myIMU.readFloatGyroZ(); // Gyro Z
    gZ = (fabsf(nextValue - gZ) > gyroscopeMargin)
         ? nextValue : 0.0f; gyroMovementZ = 1;

    // STEP4: Print New Pruned Values
    Serial.print(F("Run Number: "));
    Serial.println(runNumber);

    Serial.println();
    Serial.println();
    Serial.println(F("Accelerometer Values:"));

    Serial.print(aX, 3);
    Serial.print(',');

    Serial.print(aY, 3);
    Serial.print(',');

    Serial.print(aZ, 3);

    Serial.println();
    Serial.println();

    Serial.println(F("Gyroscope Values:"));

    Serial.print(gX, 3);
    Serial.print(',');

    Serial.print(gY, 3);
    Serial.print(',');

    Serial.print(gZ, 3);

    Serial.println();
    Serial.println();

    runNumber++;

    // STEP 5: Merges different data into a single one.


    delay(5000);

    if (samplesRead == numSamples) {
        Serial.println();
    }

    // MARK: Potentially Removable Component of Program
    // Reset Values of Contents
    accelMovementX = 0;
    gyroMovementX = 0;

    accelMovementY = 0;
    gyroMovementY = 0;

    accelMovementZ = 0;
    gyroMovementZ = 0;
  }
}

