#include "LSM6DS3.h"
#include <Wire.h>
#include <math.h>
#include <stdint.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A); // Prepare Sensor

float aX = 0.000;
float aY = 0.000;
float aZ = 0.000;

float gX = 0.000;
float gY = 0.000;
float gZ = 0.000;

void agXYZ() {
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    gX = myIMU.readFloatGyroX();
    gY = myIMU.readFloatGyroY();
    gZ = myIMU.readFloatGyroZ();
}

float aXoriginal = 0.000;
float aYoriginal = 0.000;
float aZoriginal = 0.000;

float gXoriginal = 0.000;
float gYoriginal = 0.000;
float gZoriginal = 0.000;

void agOriginalXYZ() {
    agXYZ();
    aXoriginal = aX;
    aYoriginal = aY;
    aZoriginal = aZ;

    gXoriginal = gX;
    gYoriginal = gY;
    gZoriginal = gZ;
}

void initialRunning() {
    Serial.println("Accelerometer:");
    Serial.println("The format used is: aX, aY, aZ");

    Serial.print(aX, 3);
    Serial.print(",");
    Serial.print(aY, 3);
    Serial.print(",");
    Serial.println(aZ, 3);

    delay(500);

    Serial.println("Gyroscope:");
    Serial.println("The format used is: gX, gY, gZ");

    Serial.print(gX, 3);
    Serial.print(",");
    Serial.print(gY, 3);
    Serial.print(",");
    Serial.println(gZ, 3);
}

bool sensorReadyStatus = false;

void setup() {
    Serial.begin(115200);
    if (myIMU.begin() != 0) {
        Serial.println("Sensor Intialisation Failed.");
        return;
    } else {
        Serial.println("Device Intiated Successfuly!");
        sensorReadyStatus = true;
    }

    delay(500);

    agXYZ();
    initialRunning();

    Serial.println("Initial Status of Sensors:");
}

void loop() {
    if (sensorReadyStatus == false) {
        return;
    }

}
