#include "IMU.h"

// Pin definitions
#define IMU_CS 13
#define SPI_SCK 10
#define SPI_SDI 8
#define SPI_SDO 11

// Create ICM42688 instance with SPI1 and CS pin 13
ICM42688 imu(SPI1, IMU_CS);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("ICM42688 Library Test");
    Serial.println("=====================");

    // Configure SPI pins
    SPI1.setSCK(SPI_SCK);
    SPI1.setTX(SPI_SDO);
    SPI1.setRX(SPI_SDI);
    SPI1.begin();

    // Initialize IMU
    if (imu.begin())
    {
        Serial.println("✓ IMU initialized successfully");
    }
    else
    {
        Serial.println("✗ IMU initialization failed");
    }

    Serial.println();
}

void loop()
{
    // Read sensor data
    imu.readAll();

    // Print accelerometer data
    Serial.print("Accel: ");
    Serial.print(imu.getAccelX(), 2);
    Serial.print(", ");
    Serial.print(imu.getAccelY(), 2);
    Serial.print(", ");
    Serial.print(imu.getAccelZ(), 2);

    // Print gyroscope data
    Serial.print(" | Gyro: ");
    Serial.print(imu.getGyroX(), 2);
    Serial.print(", ");
    Serial.print(imu.getGyroY(), 2);
    Serial.print(", ");
    Serial.print(imu.getGyroZ(), 2);

    // Print temperature
    Serial.print(" | Temp: ");
    Serial.print(imu.getTemp(), 2);
    Serial.println(" C");

    delay(100);
}
