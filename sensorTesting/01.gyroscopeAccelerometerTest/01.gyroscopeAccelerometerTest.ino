/*
MARK: LIBRARIES REQUIRED
Arduino_Sensorkit

*/

#include "Arduino_SensorKit.h"
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);

  while (!Serial);
  Environment.begin();
  Accelerometer.begin();
}

void loop() {
  Serial.print("Accel X: ");
  Serial.print(Accelerometer.readX());

  Serial.print(" | Gyro X: ");
  Serial.println(Gyroscope.readX());

  delay(200);
}
