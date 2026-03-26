/*
  RICE ECLIPSE AFS - IMU Debug Build (FIXED)
  Temperature + Power + ICM-42688 (QMI8658 register map) only.
  IMU driven via direct SPI register access.

  FIX SUMMARY:
  1. Removed debug reads of status registers from inside imuReadRegs() burst.
     Those extra SPI transactions while CS was low corrupted the burst read
     and confused the sensor state machine.
  2. Simplified imuReadAll() to properly follow the datasheet's Locking
     Mechanism flow:
       a. Poll STATUSINT until Avail=1 AND Locked=1
       b. Burst-read the 14 data bytes (TEMP through GZ_H)
       c. Read STATUS0 (0x2E) to release the lock for the next sample
  3. Removed the stale/broken imuReadRegs() function (burst read is now
     done inline in imuReadAll() for clarity).
*/

// ===================== LIBRARIES =====================
#include <Temperature_LM75_Derived.h>
#include <INA226.h>
#include <Wire.h>
#include <SPI.h>

// ===================== PIN DEFINITIONS =====================
#define led1 25
#define led2 5

#define current1 26
#define current2 27
#define current3 28
#define current4 29

#define PWM1 19
#define PWM2 22
#define PWM3 23
#define PWM4 24

#define I2C_SDA 20
#define I2C_SCL 21

#define SPI_SDO 11  // MOSI
#define SPI_SDI 8   // MISO
#define SPI_SCK 10

#define IMU_CS  13
#define MAG_CS  14
#define ACC_CS  15
#define BAR_CS  12

// =====================================================
//  ICM-42688 REGISTER MAP (QMI8658 compatible)
// =====================================================

// Chip info
#define REG_WHO_AM_I       0x00  // Expected: 0x05
#define REG_REVISION_ID    0x01

// Control registers
#define REG_CTRL1          0x02  // SPI mode, auto-increment, endianness
#define REG_CTRL2          0x03  // Accel: full-scale, ODR, self-test
#define REG_CTRL3          0x04  // Gyro:  full-scale, ODR, self-test
#define REG_CTRL5          0x06  // Low-pass filter settings
#define REG_CTRL7          0x08  // Sensor enable (aEN, gEN)
#define REG_CTRL9          0x0A  // Host commands

// Status
#define REG_STATUSINT      0x2D
#define REG_STATUS0        0x2E  // Data-ready flags

// Sensor data output (16-bit, two's complement)
#define REG_TEMP_L         0x33
#define REG_TEMP_H         0x34
#define REG_AX_L           0x35
#define REG_AX_H           0x36
#define REG_AY_L           0x37
#define REG_AY_H           0x38
#define REG_AZ_L           0x39
#define REG_AZ_H           0x3A
#define REG_GX_L           0x3B
#define REG_GX_H           0x3C
#define REG_GY_L           0x3D
#define REG_GY_H           0x3E
#define REG_GZ_L           0x3F
#define REG_GZ_H           0x40

// Reset
#define REG_RESET          0x60  // Write 0xB0 to soft-reset

// Expected WHO_AM_I value
#define IMU_WHO_AM_I_VAL   0x05

// CTRL1 fields
#define CTRL1_ADDR_AI      0x60  // Auto-increment ON, Little-Endian, 4-wire SPI

// CTRL2: Accel config  ±16g, ~448Hz 6DOF
#define CTRL2_16G_500HZ    0x34

// CTRL3: Gyro config   ±2048dps, ~448Hz
#define CTRL3_2048DPS_500HZ 0x74

// CTRL7: Sensor enable with SyncSample
//   Bit 7: SyncSample = 1  (locking mechanism enabled)
//   Bit 1: gEN = 1
//   Bit 0: aEN = 1
#define CTRL7_ACCEL_GYRO   0x83

// Scale factors
float accelScale = 1.0f / 2048.0f;   // g      (±16g)
float gyroScale  = 1.0f / 16.0f;     // dps    (±2048dps)
float tempScale  = 1.0f / 256.0f;    // °C

// SPI settings (1 MHz for safety during debug)
SPISettings imuSPISettings(1000000, MSBFIRST, SPI_MODE0);

// ===================== GLOBAL VARIABLES =====================
double temp;                          // LM75 board temperature
double batteryVoltage, shuntVoltage, currentDraw, powerDraw;

float imuTemp;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

bool inaReady = false;
bool imuReady = false;

// ===================== SENSOR OBJECTS =====================
Generic_LM75 temperatureSensor;
INA226 INA(0x40);

// ===================== IMU SPI HELPERS =====================

void imuWriteReg(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg & 0x7F);   // Bit 7 = 0 -> write
  SPI1.transfer(val);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
}

uint8_t imuReadReg(uint8_t reg) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg | 0x80);   // Bit 7 = 1 -> read
  uint8_t val = SPI1.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
  return val;
}

bool imuInit() {
  // --- Soft reset ---
  imuWriteReg(REG_RESET, 0xB0);
  delay(100);  // Wait for reset (max 15ms per datasheet, generous margin)

  // --- Read WHO_AM_I ---
  uint8_t whoami = imuReadReg(REG_WHO_AM_I);
  Serial.print("  WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  if (whoami != IMU_WHO_AM_I_VAL) {
    Serial.print("  Expected 0x");
    Serial.println(IMU_WHO_AM_I_VAL, HEX);
    return false;
  }

  // --- Read revision ---
  uint8_t rev = imuReadReg(REG_REVISION_ID);
  Serial.print("  Revision = 0x");
  Serial.println(rev, HEX);

  // --- CTRL1: Enable auto-increment, Little-Endian, 4-wire SPI ---
  // First write with SensorDisable=1 to ensure clock is on for config
  imuWriteReg(REG_CTRL1, 0x61);
  delay(1);
  // Then clear SensorDisable
  imuWriteReg(REG_CTRL1, CTRL1_ADDR_AI);
  delay(1);

  // Verify CTRL1
  uint8_t c1 = imuReadReg(REG_CTRL1);
  Serial.print("  CTRL1    = 0x");
  Serial.println(c1, HEX);

  // --- Configure accel: ±16g, ~448Hz (6DOF) ---
  imuWriteReg(REG_CTRL2, CTRL2_16G_500HZ);
  delay(1);

  // --- Configure gyro: ±2048 dps, ~448Hz ---
  imuWriteReg(REG_CTRL3, CTRL3_2048DPS_500HZ);
  delay(1);

  // --- Enable accel + gyro with SyncSample (locking mechanism) ---
  imuWriteReg(REG_CTRL7, CTRL7_ACCEL_GYRO);
  delay(200);  // Wait for gyro startup (~150ms) + settling

  // Verify config
  uint8_t c2 = imuReadReg(REG_CTRL2);
  uint8_t c3 = imuReadReg(REG_CTRL3);
  uint8_t c7 = imuReadReg(REG_CTRL7);
  Serial.print("  CTRL2    = 0x");
  Serial.print(c2, HEX);
  Serial.print("  (accel config, expect 0x");
  Serial.print(CTRL2_16G_500HZ, HEX);
  Serial.println(")");
  Serial.print("  CTRL3    = 0x");
  Serial.print(c3, HEX);
  Serial.print("  (gyro config,  expect 0x");
  Serial.print(CTRL3_2048DPS_500HZ, HEX);
  Serial.println(")");
  Serial.print("  CTRL7    = 0x");
  Serial.print(c7, HEX);
  Serial.print("  (sensor enable, expect 0x");
  Serial.print(CTRL7_ACCEL_GYRO, HEX);
  Serial.println(")");

  // Do an initial read of STATUS0 to clear any stale state
  imuReadReg(REG_STATUS0);

  return true;
}

/*
 * imuReadAll() - Read all sensor data using the Locking Mechanism.
 *
 * Per the QMI8658/ICM-42688 datasheet (Section: Locking Mechanism):
 *
 *   With SyncSample enabled (CTRL7.bit7 = 1):
 *
 *   1. Read STATUSINT (0x2D).
 *      - If Avail (bit 0) = 1, the locking process begins.
 *      - If Locked (bit 1) = 1, data is locked and safe to read.
 *      - If Avail=1 but Locked=0, wait Data_Lock_Delay then proceed.
 *
 *   2. Burst-read 14 bytes starting from TEMP_L (0x33) through GZ_H (0x40).
 *      Reading GZ_H (the last byte when gyro is enabled) automatically
 *      releases the lock.
 *
 *   3. Read STATUS0 (0x2E) to acknowledge the read and allow the sensor
 *      to prepare the next sample.
 *
 *   CRITICAL: Do NOT perform any other SPI register reads between
 *   triggering the lock (step 1) and completing the burst read (step 2).
 *   Doing so corrupts the data and/or permanently locks the sensor.
 */
void imuReadAll() {
  uint8_t statusInt = 0;

  // ---- Step 1: Poll STATUSINT for data availability ----
  uint32_t t0 = millis();
  while (millis() - t0 < 10) {  // 10ms timeout (generous for ~448Hz ODR)
    statusInt = imuReadReg(REG_STATUSINT);
    if (statusInt & 0x01) {
      // Avail bit is set — locking has been triggered by this read
      break;
    }
  }

  if (!(statusInt & 0x01)) {
    // Timeout: no data available
    return;
  }

  // If Locked bit is not yet set, wait the Data_Lock_Delay.
  // At ODR setting 4 (448.4Hz), the delay is 12 microseconds.
  if (!(statusInt & 0x02)) {
    delayMicroseconds(15);  // Slightly more than 12us for margin
  }

  // ---- Step 2: Burst-read 14 bytes (TEMP_L through GZ_H) ----
  uint8_t buf[14];

  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  delayMicroseconds(1);  // CS setup time

  SPI1.transfer(REG_TEMP_L | 0x80);  // Read starting at 0x33, auto-increment
  for (uint8_t i = 0; i < 14; i++) {
    buf[i] = SPI1.transfer(0x00);
  }

  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();

  // Reading through GZ_H (byte 13, register 0x40) automatically releases
  // the locking mechanism.

  // ---- Step 3: Read STATUS0 to clear data-ready flags ----
  imuReadReg(REG_STATUS0);

  // ---- Step 4: Parse the data (Little-Endian, two's complement) ----
  int16_t rawTemp = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawAx   = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawAy   = (int16_t)((buf[5] << 8) | buf[4]);
  int16_t rawAz   = (int16_t)((buf[7] << 8) | buf[6]);
  int16_t rawGx   = (int16_t)((buf[9] << 8) | buf[8]);
  int16_t rawGy   = (int16_t)((buf[11] << 8) | buf[10]);
  int16_t rawGz   = (int16_t)((buf[13] << 8) | buf[12]);

  // ---- Step 5: Apply scale factors ----
  imuTemp = (float)rawTemp * tempScale;
  accelX  = (float)rawAx   * accelScale;
  accelY  = (float)rawAy   * accelScale;
  accelZ  = (float)rawAz   * accelScale;
  gyroX   = (float)rawGx   * gyroScale;
  gyroY   = (float)rawGy   * gyroScale;
  gyroZ   = (float)rawGz   * gyroScale;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println("========================================");
  Serial.println("   RICE ECLIPSE AFS - IMU Debug (FIXED)");
  Serial.println("========================================");

  // Pin modes
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(current1, INPUT);
  pinMode(current2, INPUT);
  pinMode(current3, INPUT);
  pinMode(current4, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  // All chip selects HIGH (inactive)
  int csPins[] = {IMU_CS, MAG_CS, ACC_CS, BAR_CS};
  for (int p : csPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }

  // I2C
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  // SPI
  SPI1.setSCK(SPI_SCK);
  SPI1.setTX(SPI_SDO);
  SPI1.setRX(SPI_SDI);
  SPI1.begin();

  // --- INA226 ---
  Serial.print("[INA226]   ");
  if (!INA.begin()) {
    Serial.println("FAIL");
  } else {
    INA.setMaxCurrentShunt(15, 0.002);
    inaReady = true;
    Serial.println("OK");
  }

  // --- IMU ---
  Serial.println("[ICM42688] Initializing...");
  if (!imuInit()) {
    Serial.println("[ICM42688] FAIL");
  } else {
    imuReady = true;
    Serial.println("[ICM42688] OK");
  }

  // --- Raw register dump for debugging ---
  Serial.println();
  Serial.println("  Register dump:");
  for (uint8_t reg = 0x00; reg <= 0x0A; reg++) {
    uint8_t val = imuReadReg(reg);
    Serial.print("    0x");
    if (reg < 0x10) Serial.print("0");
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    if (val < 0x10) Serial.print("0");
    Serial.println(val, HEX);
  }
  Serial.println();

  Serial.println("========================================");
  Serial.println("   Starting loop...");
  Serial.println("========================================");
  Serial.println();

  delay(500);
}

// ===================== LOOP =====================
void loop() {
  // --- LM75 temperature ---
  temp = temperatureSensor.readTemperatureC();

  // --- INA226 power ---
  if (inaReady) {
    batteryVoltage = INA.getBusVoltage();
    currentDraw    = INA.getCurrent_mA();
    powerDraw      = INA.getPower_mW();
  }

  // --- IMU ---
  if (imuReady) {
    imuReadAll();
  }

  // --- Print ---
  Serial.println("----------------------------------------");

  Serial.print("  Temp (LM75):  ");
  Serial.print(temp, 1);
  Serial.println(" C");

  if (inaReady) {
    Serial.print("  Voltage:      ");
    Serial.print(batteryVoltage, 3);
    Serial.print(" V  |  Current: ");
    Serial.print(currentDraw, 3);
    Serial.print(" mA  |  Power: ");
    Serial.print(powerDraw, 3);
    Serial.println(" mW");
  }

  if (imuReady) {
    Serial.print("  IMU Temp:     ");
    Serial.print(imuTemp, 2);
    Serial.println(" C");

    Serial.print("  Accel (g):    X:");
    Serial.print(accelX, 3);
    Serial.print("  Y:");
    Serial.print(accelY, 3);
    Serial.print("  Z:");
    Serial.println(accelZ, 3);

    Serial.print("  Gyro (dps):   X:");
    Serial.print(gyroX, 2);
    Serial.print("  Y:");
    Serial.print(gyroY, 2);
    Serial.print("  Z:");
    Serial.println(gyroZ, 2);
  } else {
    Serial.println("  IMU:          [offline]");
  }

  Serial.println("----------------------------------------");
  Serial.println();

  delay(200);
}
