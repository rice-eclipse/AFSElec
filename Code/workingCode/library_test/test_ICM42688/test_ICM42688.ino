#include "IMU.h"
#include <SPI.h>

#define IMU_CS  13
#define MAG_CS  14
#define ACC_CS  15
#define BAR_CS  12

#define SPI_SDO 11
#define SPI_SDI 8
#define SPI_SCK 10

ICM42688 imu(SPI1, IMU_CS, ACCEL_8G, GYRO_1000DPS);

void setup() {
    Serial.begin(115200);
    delay(1000);

    int csPins[] = {IMU_CS, MAG_CS, ACC_CS, BAR_CS};
    for (int p : csPins) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }

    SPI1.begin();
    SPI1.setSCK(SPI_SCK); 
    SPI1.setTX(SPI_SDO); 
    SPI1.setRX(SPI_SDI); 
    SPI1.begin();


    if (imu.begin()) {
        Serial.println("IMU initialized successfully");
    } else {
        Serial.println("IMU initialization failed");
    }
}

void loop() {
    if(imu.readAll()){

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
    }
    delay(100);
}
