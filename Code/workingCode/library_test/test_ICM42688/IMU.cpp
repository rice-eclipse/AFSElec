#include "IMU.h"

// Constructor
ICM42688::ICM42688(SPIClass &spi, uint8_t cs_pin)
    : spi(spi), cs_pin(cs_pin), spiSettings(1000000, MSBFIRST, SPI_MODE0)
{
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    configure_accel(ACCEL_16G);   // Default to ±16g
    configure_gyro(GYRO_2000DPS); // Default to ±2000dps
}

ICM42688::ICM42688(SPIClass &spi, uint8_t cs_pin, accel_range_t accelRange, gyro_range_t gyroRange)
    : spi(spi), cs_pin(cs_pin), spiSettings(1000000, MSBFIRST, SPI_MODE0), accelRange(accelRange), gyroRange(gyroRange)
{
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    configure_accel(accelRange);
    configure_gyro(gyroRange);
}

// Write to a register
void ICM42688::writeReg(uint8_t reg, uint8_t val)
{
    spi.beginTransaction(spiSettings);
    digitalWrite(cs_pin, LOW);
    spi.transfer(reg & 0x7F);
    spi.transfer(val);
    digitalWrite(cs_pin, HIGH);
    spi.endTransaction();
}

// Read from a register
uint8_t ICM42688::readReg(uint8_t reg)
{
    spi.beginTransaction(spiSettings);
    digitalWrite(cs_pin, LOW);
    spi.transfer(reg | 0x80);
    uint8_t val = spi.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    spi.endTransaction();
    return val;
}

// Initialize the sensor
bool ICM42688::begin()
{
    // Reset sensor
    writeReg(REG_RESET, 0xB0);
    delay(100);

    // Check WHO_AM_I register
    if (readReg(REG_WHO_AM_I) != 0x05)
    {
        return false;
    }

    // Configure sensor
    writeReg(REG_CTRL1, 0x60); // auto increment on, MSB on
    writeReg(REG_CTRL2, 0x34); // ±16g - accelerometer settings
    writeReg(REG_CTRL3, 0x74); // ±2048dps - gyroscope settings

    // Initialize FIFO in Stream Mode (0x02) to stream data straight to registers
    writeReg(REG_FIFO_CTRL, 0x02);

    // Enable sensors
    writeReg(REG_CTRL7, 0x03);
    delay(50);

    return true;
}

void ICM42688::configure_accel(accel_range_t index)
{
    uint8_t accelConfig = index;

    writeReg(REG_CTRL2, accelConfig);
}

void ICM42688::configure_gyro(gyro_range_t index)
{
    uint8_t gyroConfig = index;

    writeReg(REG_CTRL3, gyroConfig);
}

// Read all sensor data
void ICM42688::readAll()
{
    // 1. Check if samples are available in FIFO
    uint8_t countL = readReg(REG_FIFO_SMPL_L);
    uint8_t status = readReg(REG_FIFO_STATUS);
    uint16_t samplesAvail = ((status & 0x03) << 8) | countL;

    if (samplesAvail > 0)
    {
        // 2. CTRL9 Handshake: Enable FIFO Read Mode
        writeReg(REG_CTRL9, 0x05);

        // 3. Burst Read Data from registers since we are in Stream mode
        uint8_t buf[12];
        spi.beginTransaction(spiSettings);
        digitalWrite(cs_pin, LOW);

        spi.transfer(0x35 | 0x80); // start of burst read
        for (int i = 0; i < 12; i++)
        {
            buf[i] = spi.transfer(0x00);
        }
        digitalWrite(cs_pin, HIGH);
        spi.endTransaction();

        // 4. Release FIFO: Reset FIFO_CTRL to clear RD_MODE (Bit 7)
        writeReg(REG_FIFO_CTRL, 0x02);

        // 5. Process Little-Endian Data
        accelX = (int16_t)((buf[1] << 8) | buf[0]) * accelScale;
        accelY = (int16_t)((buf[3] << 8) | buf[2]) * accelScale;
        accelZ = (int16_t)((buf[5] << 8) | buf[4]) * accelScale;
        gyroX = (int16_t)((buf[7] << 8) | buf[6]) * gyroScale;
        gyroY = (int16_t)((buf[9] << 8) | buf[8]) * gyroScale;
        gyroZ = (int16_t)((buf[11] << 8) | buf[10]) * gyroScale;

        // Read Temperature separately (non-FIFO register)
        int16_t rawTemp = (int16_t)((readReg(REG_TEMP_H) << 8) | readReg(REG_TEMP_L));
        imuTemp = (float)rawTemp * tempScale;
    }
}
