/*
  RICE ECLIPSE AFS - IMU Debug Build
  Temperature + Power + ICM-42688 (QMI8658 register map) only.
  IMU driven via direct SPI register access.
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
//  Based on the provided datasheet
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
// With auto-increment, burst read from TEMP_L through GZ_H = 14 bytes
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
//   Bit 7: SIM  (0=4-wire SPI)
//   Bit 6: ADDR_AI (1=auto-increment)
//   Bit 5: BE   (0=Little-Endian, 1=Big-Endian)
//   Bit 0: SensorDisable (0=clock on)
// FIX: was 0x60 which set BE=1 (Big-Endian), corrected to 0x40 (BE=0, Little-Endian)
#define CTRL1_ADDR_AI      0x40  // Auto-increment ON, Little-Endian, 4-wire SPI

// CTRL2: Accel config
//   Bits [6:4] aFS:  000=±2g, 001=±4g, 010=±8g, 011=±16g
//   Bits [3:0] aODR: 0011=1000Hz(accel-only)/896.8Hz(6DOF),
//                     0100=500/448.4, 0101=250/224.2, 0110=125/112.1
#define CTRL2_16G_500HZ    0x34  // aFS=011(±16g), aODR=0100(~448Hz 6DOF)

// CTRL3: Gyro config
//   Bits [6:4] gFS:  000=±16dps, ..., 111=±2048dps
//   Bits [3:0] gODR: 0100=448.4Hz, 0110=112.1Hz
#define CTRL3_2048DPS_500HZ 0x74  // gFS=111(±2048dps), gODR=0100(448.4Hz)

// CTRL7: Sensor enable
//   Bit 7: SyncSample (1=locking mechanism, FIFO disabled)
//   Bit 0: aEN (accel enable)
//   Bit 1: gEN (gyro enable)
// FIX: was 0x83 which set SyncSample=1 (locking mode). The locking unlock
//      sequence was never completed correctly so the sensor froze after one read.
//      Using Non-SyncSample mode (bit7=0) allows simple STATUS0 polling.
#define CTRL7_ACCEL_GYRO   0x03  // SyncSample=0, aEN=1, gEN=1

// Scale factors
//   Accel ±16g  -> 2048 LSB/g
//   Gyro ±2048 dps -> 16 LSB/dps
//   Temp -> raw / 256 °C
float accelScale = 1.0f / 2048.0f;   // g
float gyroScale  = 1.0f / 16.0f;     // dps
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

void imuReadRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(startReg | 0x80);  // Read with auto-increment
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = SPI1.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
}

bool imuInit() {
  // --- Soft reset ---
  imuWriteReg(REG_RESET, 0xB0);
  delay(100);  // Wait for reset (max 15ms per datasheet)

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
  // Write with SensorDisable=1 first to safely reconfigure, then clear it
  imuWriteReg(REG_CTRL1, 0x41);  // ADDR_AI=1, BE=0, SensorDisable=1
  delay(1);
  imuWriteReg(REG_CTRL1, CTRL1_ADDR_AI);  // ADDR_AI=1, BE=0, SensorDisable=0
  delay(1);

  // Verify CTRL1
  uint8_t c1 = imuReadReg(REG_CTRL1);
  Serial.print("  CTRL1    = 0x");
  Serial.println(c1, HEX);

  // // --- Disable sensors before config (CTRL7 = 0x00) ---
  // imuWriteReg(REG_CTRL7, 0x00);
  // delay(1);

  // --- Configure accel: ±16g, ~448Hz (6DOF) ---
  imuWriteReg(REG_CTRL2, CTRL2_16G_500HZ);
  delay(1);

  // --- Configure gyro: ±2048 dps, ~448Hz ---
  imuWriteReg(REG_CTRL3, CTRL3_2048DPS_500HZ);
  delay(1);

  // --- Enable accel + gyro ---
  
  imuWriteReg(REG_CTRL7, CTRL7_ACCEL_GYRO);
  delay(50);  // Wait for sensors to stabilize

  


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

  return true;
}

// Burst-read 14 bytes: temp(2) + accel(6) + gyro(6)
// Little-Endian (BE=0): register order is L byte first, H byte second.
// Uses Non-SyncSample mode: poll STATUS0 for aDA+gDA, then read directly.
// No locking/unlock handshake needed in this mode.
void imuReadAll() {
  // Poll STATUS0 (0x2E) for new data flags:
  //   bit 0 = aDA (accel data available)
  //   bit 1 = gDA (gyro  data available)
  // At 448 Hz ODR new data arrives every ~2.2 ms; use a 10 ms timeout.
  uint8_t status0 = 0;
  uint32_t timeout = micros();
  while (micros() - timeout < 10000) {
    status0 = imuReadReg(REG_STATUS0);
    if ((status0 & 0x03) == 0x03) break;  // Both accel and gyro ready
  }

  if ((status0 & 0x03) == 0) {
    Serial.print("IMU: data timeout, STATUS0=0x");
    Serial.println(status0, HEX);
    return;
  }

  // Burst-read 14 bytes starting at TEMP_L (0x33) through GZ_H (0x40)
  uint8_t buf[14];
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  delayMicroseconds(1);
  SPI1.transfer(REG_TEMP_L | 0x80);
  for (uint8_t i = 0; i < 14; i++) {
    buf[i] = SPI1.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();

  // Parse Little-Endian pairs (L byte at even index, H byte at odd index)
  // FIX: rawAx was (buf[2]<<8|buf[3]) which swapped H and L — corrected below
  int16_t rawTemp = (int16_t)((buf[1]  << 8) | buf[0]);
  int16_t rawAx   = (int16_t)((buf[3]  << 8) | buf[2]);
  int16_t rawAy   = (int16_t)((buf[5]  << 8) | buf[4]);
  int16_t rawAz   = (int16_t)((buf[7]  << 8) | buf[6]);
  int16_t rawGx   = (int16_t)((buf[9]  << 8) | buf[8]);
  int16_t rawGy   = (int16_t)((buf[11] << 8) | buf[10]);
  int16_t rawGz   = (int16_t)((buf[13] << 8) | buf[12]);

  // Apply scaling
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
  Serial.println("   RICE ECLIPSE AFS - IMU Debug");
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
  delayMicroseconds(100);
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