#ifndef IMU_H
#define IMU_H

#include <SPI.h>

// ===================== REGISTER MAP =====================
#define REG_WHO_AM_I 0x00
#define REG_CTRL1 0x02
#define REG_CTRL2 0x03
#define REG_CTRL3 0x04
#define REG_CTRL7 0x08
#define REG_CTRL9 0x0A       // Host command register
#define REG_FIFO_CTRL 0x14   // FIFO Control (Release)
#define REG_FIFO_SMPL_L 0x15 // FIFO sample count LSB
#define REG_FIFO_STATUS 0x16 // FIFO sample count MSB
#define REG_FIFO_DATA 0x17   // FIFO data port
#define REG_STATUSINT 0x2D
#define REG_TEMP_L 0x33
#define REG_TEMP_H 0x34
#define REG_RESET 0x60

typedef enum
{
    ACCEL_2G = 0b00000000,
    ACCEL_4G = 0b00010000,
    ACCEL_8G = 0b0010000,
    ACCEL_16G = 0b00110000
} accel_range_t;

typedef enum
{
    GYRO_250DPS = 0b00000000,
    GYRO_500DPS = 0b00010000,
    GYRO_1000DPS = 0b00100000,
    GYRO_2000DPS = 0b00110000
} gyro_range_t;

class ICM42688
{
public:
    // Constructor: Pass in SPI object and CS pin
    ICM42688(SPIClass &spi, uint8_t cs_pin);
    ICM42688(SPIClass &spi, uint8_t cs_pin, accel_range_t accelRange, gyro_range_t gyroRange);

    // Initialize the sensor
    bool begin();

    void configure_accel(accel_range_t index);

    void configure_gyro(gyro_range_t index);

    // Read all sensor data
    bool readAll();

    // Getters for sensor data
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    float getTemp() const { return imuTemp; }

private:
    SPIClass &spi;
    uint8_t cs_pin;
    SPISettings spiSettings;
    accel_range_t accelRange;
    gyro_range_t gyroRange;

    // Sensor scales
    float accelScale = 1.0f / 2048.0f;
    float gyroScale = 1.0f / 16.0f;
    float tempScale = 1.0f / 256.0f;

    // Sensor data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float imuTemp;

    // Internal methods
    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
};

#endif