#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// ================= MARK: Program Initialization =================
// Initialize Sensors
LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool imuReady = false;

// Active Sensor Values
float aX = 0.0f;
float aY = 0.0f;
float aZ = 0.0f;

float gX = 0.0f;
float gY = 0.0f;
float gZ = 0.0f;

// Prev Accel Values
float previousAccel[3] = {
    0.0f, 0.0f, 0.0f
};

// ================= MARK: Sensor THRESHOLDS =================
// Accelerometer delta threshold for valid reading (G) - Aka measurable change in 9.8m/s
constexpr float accelerationMargin = 0.15f;

// Gyroscope change in degrees for valid reading (DPS)
constexpr float gyroscopeMargin = 0.5f;

// Stable readings till classified
constexpr uint8_t positionWindowSamples = 3;

// Maximum distance in G-force from Accelerometer
constexpr float maxPositionDistance = 0.20f;

// Minimum change in G-force from Accelerometer
constexpr float minPositionSeparation = 0.05f;

// 2 Decimal Place Rounder
constexpr float roundingFactor = 100.0f;

// ================= MARK: Runtime Variables =================
// Run Counter
uint32_t runNumber = 1;

// Changes in Gyroscope in Degrees
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

// Accel and Gyro Total Sum
float accelSum[3] = {
    0.0f, 0.0f, 0.0f
};

float gyroSum[3] = {
    0.0f, 0.0f, 0.0f
};

// Consecutive Stable Readings Counter Variable
uint8_t settledSampleCount = 0;

// Distances Classifications
float nearestPositionDistance = -1.0f;
float secondPositionDistance = -1.0f;

// Gyroscope Calibration
constexpr uint16_t calibrationSamples = 100;
constexpr unsigned long calibrationDelay = 5UL;

// ================= MARK: Movement Definitions =================
enum Movement : uint8_t {
    FORWARD_LEFT_UP,
    FORWARD_LEFT_STRAIGHT,
    FORWARD_DOWN_STRAIGHT,
    BACK_DOWN_DOWN,
    UNKNOWN
};

Movement currentMovement = UNKNOWN;

// ================= MARK: Movement Signatures =================

struct MovementSignature {
    Movement movement;
    bool enabled;
    float accel[3];
};

MovementSignature movementSignatures[] = {
    // Forward + Left + Up
    {FORWARD_LEFT_UP, false,
        {0.00f, 0.00f, 0.00f}},

    // Forward + Left + Straight
    {FORWARD_LEFT_STRAIGHT, false,
        {0.00f, 0.00f, 0.00f}},

    // Forward + Down + Straight
    {FORWARD_DOWN_STRAIGHT, false,
        {0.00f, 0.00f, 0.00f}},

    // Backward + Down + Down
    {BACK_DOWN_DOWN, false,
        {0.00f, 0.00f, 0.00f}}
};

// Counter for amount of signatures that matches
constexpr uint8_t movementSignatureCount =
    sizeof(movementSignatures) / sizeof(movementSignatures[0]);

// ================= MARK: Utility Purpose Functions =================
// Round to 2dp using float to float over doubles
float roundTo2dp(float value) {
    return roundf(value * roundingFactor) / roundingFactor;
}

// Calculate the magnitude using Euclidean distance 3-Dimensional
float vectorMagnitude(const float values[3]) {
    return sqrtf(
        (values[0] * values[0]) +
        (values[1] * values[1]) +
        (values[2] * values[2])
    );
}

// Abstract Values from Sensors
int8_t getDirection(float value, float threshold) {
    if (value > threshold) {
        return 1;
    }
    if (value < -threshold) {
        return -1;
    }
    return 0;
}

// Movement Serial Print
const char* movementName(Movement movement) {
    switch (movement) {
        case FORWARD_LEFT_UP:
            return "FORWARD_LEFT_UP";
        case FORWARD_LEFT_STRAIGHT:
            return "FORWARD_LEFT_STRAIGHT";
        case FORWARD_DOWN_STRAIGHT:
            return "FORWARD_DOWN_STRAIGHT";
        case BACK_DOWN_DOWN:
            return "BACK_DOWN_DOWN";
        default:
            return "UNKNOWN";
    }
}

// Reset Accumulator Variables
void resetPositionWindow() {
    for (uint8_t i = 0; i < 3; i++) {
        accelSum[i] = 0.0f;
        gyroSum[i] = 0.0f;
    }
    settledSampleCount = 0;
}

// ================= MARK: Gyroscope Calibration =================
void calibrateGyroscope() {

    // Reset Values
    gyroOffsetX = 0.0f;
    gyroOffsetY = 0.0f;
    gyroOffsetZ = 0.0f;

    Serial.println();
    Serial.println("Starting gyroscope calibration...");

    for (uint16_t i = 0; i < calibrationSamples; i++) {

        gyroOffsetX += myIMU.readFloatGyroX();
        gyroOffsetY += myIMU.readFloatGyroY();
        gyroOffsetZ += myIMU.readFloatGyroZ();

        delay(calibrationDelay);
    }

    // Average to find sensor bias
    gyroOffsetX /= calibrationSamples;
    gyroOffsetY /= calibrationSamples;
    gyroOffsetZ /= calibrationSamples;

    Serial.println("Gyroscope calibration complete.");
    Serial.print("Offset X: ");
    Serial.println(gyroOffsetX, 4);
    Serial.print("Offset Y: ");
    Serial.println(gyroOffsetY, 4);
    Serial.print("Offset Z: ");
    Serial.println(gyroOffsetZ, 4);
    Serial.println();
}

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

