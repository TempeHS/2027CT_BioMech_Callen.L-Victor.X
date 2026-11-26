#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// ================================== MARK: VARIABLE INITIALISATION ==================================

// ================= MARK: Program Initialization =================
// Debugging Speed
constexpr unsigned long pruneReadingDiff = 500UL;

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
constexpr float accelerationMargin = 0.35f;

// Gyroscope change in degrees for valid reading (DPS)
constexpr float gyroscopeMargin = 2.0f;

// Stable readings till classified
constexpr uint8_t positionWindowSamples = 3;

// Maximum distance in G-force from Accelerometer
constexpr float maxPositionDistance = 1.0f;

// Minimum distance difference between the closest two signatures (G)
constexpr float minPositionSeparation = 0.02f;

// 2 Decimal Place Rounder
constexpr float roundingFactor = 100.0f;

// Limit on Accel Vector Change in Samples
constexpr float positionAccelChangeLimit = 0.4f;

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
        {0.05f, 0.05f, 0.10f}},

    // Forward + Left + Straight
    {FORWARD_LEFT_STRAIGHT, false,
        {0.05f, -0.05f, 0.10f}},

    // Forward + Down + Straight
    {FORWARD_DOWN_STRAIGHT, false,
        {0.05f, 0.05f, 0.10f}},

    // Backward + Down + Down
    {BACK_DOWN_DOWN, false,
        {-0.05f, -0.05f, -0.05f}}
};

// Counter for amount of signatures that matches
constexpr uint8_t movementSignatureCount =
    sizeof(movementSignatures) / sizeof(movementSignatures[0]);

// ================= MARK: Hand Stable =================
// Calculate the magnitude using Euclidean distance 3-Dimensional
float vectorMagnitude(const float values[3]) {
    return sqrtf(
        (values[0] * values[0]) +
        (values[1] * values[1]) +
        (values[2] * values[2])
    );
}

bool isHandSettled(
    const float deltaAccel[3],
    const float gyro[3],
    float& accelChangeMagnitude,
    float& gyroMagnitude
) {
    // Calculate total accelerometer change
    accelChangeMagnitude = vectorMagnitude(deltaAccel);

    // Calculate total angular velocity
    gyroMagnitude = vectorMagnitude(gyro);

    // Are Both Satisfied
    bool accelerationStable =
        accelChangeMagnitude <= positionAccelChangeLimit;
    bool gyroscopeStable =
        gyroMagnitude <= gyroscopeMargin;
    return accelerationStable && gyroscopeStable;
}

// ================= MARK: Utility Purpose Functions =================
// Round to 2dp using float to float over doubles
float roundTo2dp(float value) {
    return roundf(value * roundingFactor) / roundingFactor;
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

// ================= MARK: UPDATED Classification =================
Movement classifyMovement(
    const float accel[3], // For absolute
    const float gyro[3], // For Bias
    float& bestDistanceOut, // Closest Sig
    float& secondDistanceOut // Second Closest Sig
) {
    // Start Output Distances
    bestDistanceOut = -1.0f; // Show unstable because Euclidean can't be negative
    secondDistanceOut = -1.0f;

    // STEP 1: Did Arm Stop?
    float gyroMagnitude = vectorMagnitude(gyro); // Wait til arm stopped using G=(G^2x + G^2y + G^2z) (looks for stability)
    if (gyroMagnitude > gyroscopeMargin) {
        return UNKNOWN;
    }

    // STEP 2: Start nearest-match variables
    float bestDistance = 0.0f;
    float secondBestDistance = 0.0f;
    Movement bestMovement = UNKNOWN;
    bool foundBest = false;
    bool foundSecondBest = false;

    // STEP 3: See which signatures are enabled
    for (uint8_t i = 0; i < movementSignatureCount; i++) {
        const MovementSignature& signature =
            movementSignatures[i];

        if (!signature.enabled) {
            continue;
        }

        // STEP 4: Calculate Euclidean distance
        float dx = accel[0] - signature.accel[0];
        float dy = accel[1] - signature.accel[1];
        float dz = accel[2] - signature.accel[2];

        float distance = sqrtf(
            (dx * dx) +
            (dy * dy) +
            (dz * dz)
        );

        // STEP 5: Find closest and 2nd closest.
        if (!foundBest || distance < bestDistance) {
            if (foundBest) {
                secondBestDistance = bestDistance;
                foundSecondBest = true;
            }
            bestDistance = distance;
            bestMovement = signature.movement;
            foundBest = true;
        }
        else if (
            !foundSecondBest ||
            distance < secondBestDistance
        ) {
            secondBestDistance = distance;
            foundSecondBest = true;
        }
    }

    // STEP 6: Any signatures available?
    if (!foundBest) {
        return UNKNOWN;
    }

    // Store distances for Serial
    bestDistanceOut = bestDistance;
    if (foundSecondBest) {
        secondDistanceOut = secondBestDistance;
    }

    // STEP 7: Reject too much distant matches
    if (bestDistance > maxPositionDistance) {
        return UNKNOWN;
    }

    // STEP 8: Reject ambiguous matches
    if (
        foundSecondBest &&
        (secondBestDistance - bestDistance) < minPositionSeparation
    ) {
        return UNKNOWN;
    }

    // STEP 9: Provide Final Classification
    return bestMovement;
}

// ================================== MARK: PROGRAM SETUP INITIALISATION ==================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
    imuReady = false;
      return;
  } else {
    imuReady = true;
    Serial.println("Device Successfully Linked.");
    delay(2000);

    // Gyroscope Calibration for Bias
    Serial.println();
    Serial.println("Keep the hand stationary.");
    Serial.println("Gyroscope calibration starting soon...");
    delay(2000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);

    // Begin gyroscope calibration
    calibrateGyroscope();
    delay(500);

    // Initialise Sensor Orientation
    previousAccel[0] = myIMU.readFloatAccelX();
    previousAccel[1] = myIMU.readFloatAccelY();
    previousAccel[2] = myIMU.readFloatAccelZ();

    // Success Message
    Serial.println("Gyroscope Calibration Completed!");
    delay(2000);

    // Current Values from Gyro
    Serial.print("Gyro Offset X: ");
    Serial.println(gyroOffsetX, 4);
    Serial.print("Gyro Offset Y: ");
    Serial.println(gyroOffsetY, 4);
    Serial.print("Gyro Offset Z: ");
    Serial.println(gyroOffsetZ, 4);

    delay(2500);
  }
}

void loop() {
  // See if Sensor Access Working
  if (!imuReady) {
      delay(1000);
      return;
  }

  // Wait for sensor reading
  delay(pruneReadingDiff);

  // Read absolute
  float currentAccel[3] = {

      myIMU.readFloatAccelX(),
      myIMU.readFloatAccelY(),
      myIMU.readFloatAccelZ()
  };

  // Read and correct gyro values
  float currentGyro[3] = {

      myIMU.readFloatGyroX() - gyroOffsetX,
      myIMU.readFloatGyroY() - gyroOffsetY,
      myIMU.readFloatGyroZ() - gyroOffsetZ
  };

  // Update global sensor readings
  aX = currentAccel[0];
  aY = currentAccel[1];
  aZ = currentAccel[2];

  gX = currentGyro[0];
  gY = currentGyro[1];
  gZ = currentGyro[2];

  // Accel Changes
  float deltaAccel[3] = {

      currentAccel[0] - previousAccel[0],
      currentAccel[1] - previousAccel[1],
      currentAccel[2] - previousAccel[2]
  };

  // Update accelerometer readings
  for (uint8_t i = 0; i < 3; i++) {
      previousAccel[i] = currentAccel[i];
  }

  // Arm settled
  float accelChangeMagnitude = 0.0f;
  float gyroMagnitude = 0.0f;

  bool handSettled = isHandSettled(
      deltaAccel,
      currentGyro,
      accelChangeMagnitude,
      gyroMagnitude
  );

  // Smaller readings for comparison
  int8_t directions[6] = {
      getDirection(deltaAccel[0], accelerationMargin),
      getDirection(deltaAccel[1], accelerationMargin),
      getDirection(deltaAccel[2], accelerationMargin),

      getDirection(currentGyro[0], gyroscopeMargin),
      getDirection(currentGyro[1], gyroscopeMargin),
      getDirection(currentGyro[2], gyroscopeMargin)
  };

  // Use stable sensor readings
  bool classificationUpdated = false;
  float averagedAccel[3] = {
      0.0f, 0.0f, 0.0f
  };
  float averagedGyro[3] = {
      0.0f, 0.0f, 0.0f
  };

  if (!handSettled) {
      // What movement happened
      currentMovement = UNKNOWN;
      nearestPositionDistance = -1.0f;
      secondPositionDistance = -1.0f;

      // Discard previous stable readings
      resetPositionWindow();
  }
  else {
      // Add the stable readings
      for (uint8_t i = 0; i < 3; i++) {
          accelSum[i] += currentAccel[i];
          gyroSum[i] += currentGyro[i];
      }
      settledSampleCount++;


      // Enough Stable Readings?
      if (settledSampleCount >= positionWindowSamples) {
          // Calculate average readings
          for (uint8_t i = 0; i < 3; i++) {
              averagedAccel[i] = roundTo2dp(
                  accelSum[i] / positionWindowSamples
              );
              averagedGyro[i] =
                  gyroSum[i] / positionWindowSamples;
          }

          // Variable for the averaged static orientation
          currentMovement = classifyMovement(
              averagedAccel,
              averagedGyro,
              nearestPositionDistance,
              secondPositionDistance
          );
          classificationUpdated = true;

          // Reset for the next classification
          resetPositionWindow();
      }
  }

  // PRINT OUTPUTS
  Serial.println();
  Serial.println("==================================");

  Serial.print("Run Number: ");
  Serial.println(runNumber);


  // Absolute accelerometer orientation
  Serial.println();
  Serial.println("[ Accelerometer Orientation ]");
  Serial.print("X: ");
  Serial.println(currentAccel[0], 3);
  Serial.print("Y: ");
  Serial.println(currentAccel[1], 3);
  Serial.print("Z: ");
  Serial.println(currentAccel[2], 3);

  // Accelerometer changes
  Serial.println();
  Serial.println("[ Accelerometer Delta ]");
  Serial.print("Raw: ");
  Serial.print(deltaAccel[0], 3);
  Serial.print(", ");
  Serial.print(deltaAccel[1], 3);
  Serial.print(", ");
  Serial.println(deltaAccel[2], 3);

  Serial.print("Simplified: ");
  Serial.print(static_cast<int>(directions[0]));
  Serial.print(", ");
  Serial.print(static_cast<int>(directions[1]));
  Serial.print(", ");
  Serial.println(static_cast<int>(directions[2]));
  Serial.print("Vector Change: ");
  Serial.print(accelChangeMagnitude, 4);
  Serial.println(" G");

  // Corrected gyroscope readings
  Serial.println();
  Serial.println("[ Gyroscope ]");
  Serial.print("Corrected: ");
  Serial.print(currentGyro[0], 3);
  Serial.print(", ");
  Serial.print(currentGyro[1], 3);
  Serial.print(", ");
  Serial.println(currentGyro[2], 3);
  Serial.print("Simplified: ");
  Serial.print(static_cast<int>(directions[3]));
  Serial.print(", ");
  Serial.print(static_cast<int>(directions[4]));
  Serial.print(", ");
  Serial.println(static_cast<int>(directions[5]));
  Serial.print("Angular Velocity Magnitude: ");
  Serial.print(gyroMagnitude, 4);
  Serial.println(" DPS");

  // Stability
  Serial.println();
  Serial.println("[ Stability ]");
  Serial.print("Acceleration: ");
  Serial.println(
      accelChangeMagnitude <= positionAccelChangeLimit
      ? "STABLE"
      : "UNSTABLE"
  );
  Serial.print("Gyroscope: ");
  Serial.println(
      gyroMagnitude <= gyroscopeMargin
      ? "STABLE"
      : "UNSTABLE"
  );
  Serial.print("Overall: ");
  Serial.println(handSettled ? "SETTLED" : "MOVING");
  Serial.print("Stable Reading Window: ");
  Serial.print(
      classificationUpdated
      ? positionWindowSamples
      : settledSampleCount
  );
  Serial.print(" / ");
  Serial.println(positionWindowSamples);

  // Classification information
  Serial.println();
  Serial.println("[ Position Classification ]");
  Serial.print("Movement: ");
  Serial.println(movementName(currentMovement));
  Serial.print("Classification Updated: ");
  Serial.println(classificationUpdated ? "YES" : "NO");
  Serial.print("Nearest Distance: ");
  if (nearestPositionDistance >= 0.0f) {
      Serial.print(nearestPositionDistance, 4);
      Serial.println(" G");
  }
  else {
      Serial.println("N/A");
  }
  Serial.print("Second Nearest Distance: ");
  if (secondPositionDistance >= 0.0f) {
      Serial.print(secondPositionDistance, 4);
      Serial.println(" G");
  }

  else {
      Serial.println("N/A");
  }

  // Print Average Classification
  if (classificationUpdated) {
      Serial.println();
      Serial.println("[ Average Position Input ]");
      Serial.print("Accel: ");
      Serial.print(averagedAccel[0], 2);
      Serial.print(", ");
      Serial.print(averagedAccel[1], 2);
      Serial.print(", ");
      Serial.println(averagedAccel[2], 2);

      Serial.print("Gyro: ");
      Serial.print(averagedGyro[0], 3);
      Serial.print(", ");
      Serial.print(averagedGyro[1], 3);
      Serial.print(", ");
      Serial.println(averagedGyro[2], 3);
      Serial.println("Classification Window Complete.");
  }

  Serial.println("==================================");

  // Update Counter
  runNumber++;
}

