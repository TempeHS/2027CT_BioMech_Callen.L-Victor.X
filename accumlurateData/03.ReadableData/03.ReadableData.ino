#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

float aX, aY, aZ, gX, gY, gZ;

// Accelerometer Min Thresholds
const float accelerationThreshold = 2.5; // MIN. Combined Acceleration required to trigger recording
const int numSamples = 119;              // Initialisation of 119 as sample
int samplesRead = numSamples;            // Sample Read matching numSample above.

// Sensors Margin of Change
constexpr float accelerationMargin = 0.15f;  // G-Force (≈9.8m/s)
constexpr float gyroscopeMargin = 0.15f;     // Degrees per second

// Key Values
bool accelMovementX = 0;
bool gyroMovementX = 0;

bool accelMovementY = 0;
bool gyroMovementY = 0;

bool accelMovementZ = 0;
bool gyroMovementZ = 0;

// Delay for Next Reading prior to Pruning (in ms)
unsigned int pruneReadingDiff = 200;

// Run no. counter
unsigned int runNumber = 1;

// Gyroscope Baseline FOR calibration
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

// Gyro Sensor Calibration for bias
void calibrateGyroscope() {
    const int calibrationSamples = 100; // Take 100 Samples
    for (int i = 0; i < calibrationSamples; i++) {
        gyroOffsetX += myIMU.readFloatGyroX(); // Adds 100 of Each Variable
        gyroOffsetY += myIMU.readFloatGyroY();
        gyroOffsetZ += myIMU.readFloatGyroZ();
        delay(5);
    }
    gyroOffsetX /= calibrationSamples; // Calculates Average of Each
    gyroOffsetY /= calibrationSamples;
    gyroOffsetZ /= calibrationSamples;
}

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

// Set up getDirection var making the data merger simpler
int8_t getDirection(float value, float threshold) { // Use of int8_t (8 bit) over int (min. 16 bit) to save memory
    if (value > threshold)  return 1; // When value greater than threshold, set to one
    if (value < -threshold) return -1; // When value lower then threshold, set to negative one
    return 0; // Otherwise return 0
}

// Translates 6 raw sensor data into a single movement
constexpr int8_t ANY_AXIS = 2;
struct MovementSignature {
    Movement movement;
    bool enabled;
    int8_t axes[6];
};

// Temporary Movement Signatures for Each Component
MovementSignature movementSignatures[] = {
    {FORWARD_LEFT_UP, false,
        {ANY_AXIS, ANY_AXIS, ANY_AXIS,
         ANY_AXIS, ANY_AXIS, ANY_AXIS}},

    {FORWARD_LEFT_STRAIGHT, false,
        {ANY_AXIS, ANY_AXIS, ANY_AXIS,
         ANY_AXIS, ANY_AXIS, ANY_AXIS}},

    {FORWARD_DOWN_STRAIGHT, false,
        {ANY_AXIS, ANY_AXIS, ANY_AXIS,
         ANY_AXIS, ANY_AXIS, ANY_AXIS}},

    {BACK_DOWN_DOWN, false,
        {ANY_AXIS, ANY_AXIS, ANY_AXIS,
         ANY_AXIS, ANY_AXIS, ANY_AXIS}}
};

// Data Merger COmponent
Movement classifyMovement(const int8_t directions[6]) {
    // Checks if all the axis are stable
    bool stationary = true;
    for (int i = 0; i < 6; i++) {
        if (directions[i] != 0) {
            stationary = false;
            break;
        }
    }
    if (stationary) {
        return STATIONARY;
    }

    // Compare against the existing movement signatures
    for (const MovementSignature& signature : movementSignatures) {
        if (!signature.enabled) {
            continue;
        }
        bool match = true;
        for (int i = 0; i < 6; i++) {
            if (
                signature.axes[i] != ANY_AXIS &&
                signature.axes[i] != directions[i]
            ) {
                match = false;
                break;
            }
        }
        if (match) {
            return signature.movement;
        }
    }
    // IF none match, return UNKNOWN.
    return UNKNOWN;
}
// Auto Determine a Array's Element Count at Compile Time
constexpr int movementSignatureCount =
    sizeof(movementSignatures) / sizeof(movementSignatures[0]);

// Just here to display movement :)
const char* movementName(Movement movement) {
    switch (movement) {
        case STATIONARY:
            return "STATIONARY";
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

    // STEP 0: Initialise the local variables needed in this program
    float nextValue = 0;

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

    /*
    MAJOR ADJUSTMENTS
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
    */
  }
}

