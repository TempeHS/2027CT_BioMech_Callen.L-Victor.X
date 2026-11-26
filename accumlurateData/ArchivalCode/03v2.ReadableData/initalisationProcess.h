void initialRunning() {
    delay(1000);

    Serial.println("Accelerometer:");
    Serial.println("The format used is: aX, aY, aZ");

    Serial.print(aX, 3);
    Serial.print(",");
    Serial.print(aY, 3);
    Serial.print(",");
    Serial.println(aZ, 3);

    delay(1000);

    Serial.println("Gyroscope:");
    Serial.println("The format used is: gX, gY, gZ");

    Serial.print(gX, 3);
    Serial.print(",");
    Serial.print(gY, 3);
    Serial.print(",");
    Serial.println(gZ, 3);
}
