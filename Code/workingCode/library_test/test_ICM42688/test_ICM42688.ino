#include "IMU.h"
#include <SPI.h>

#define IMU_CS 13

ICM42688 imu(SPI, IMU_CS, ACCEL_8G, GYRO_1000DPS);

void setup() {
    Serial.begin(115200);
    delay(1000);

    SPI.begin();

    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    if (imu.begin()) {
        Serial.println("IMU initialized successfully");
    } else {
        Serial.println("IMU initialization failed");
    }
}

void loop() {
    imu.readAll();

    Serial.print("Accel: ");
    Serial.print(imu.getAccelX(), 2);
    Serial.print(", ");
    Serial.print(imu.getAccelY(), 2);
    Serial.print(", ");
    Serial.print(imu.getAccelZ(), 2);

    Serial.print(" | Gyro: ");
    Serial.print(imu.getGyroX(), 2);
    Serial.print(", ");
    Serial.print(imu.getGyroY(), 2);
    Serial.print(", ");
    Serial.print(imu.getGyroZ(), 2);

    Serial.print(" | Temp: ");
    Serial.print(imu.getTemp(), 2);
    Serial.println(" C");

    delay(100);
}
