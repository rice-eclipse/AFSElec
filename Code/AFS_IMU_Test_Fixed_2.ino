/*
  RICE ECLIPSE AFS - IMU Test Build
  Minimal test to verify IMU data is updating.
  
  KEY CHANGE: Using Non-SyncSample mode (CTRL7.bit7 = 0).
  This avoids the locking mechanism entirely and just polls
  STATUS0 for data-ready, then reads the data registers.
*/

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

#define SPI_SDO 11
#define SPI_SDI 8
#define SPI_SCK 10

#define IMU_CS  13
#define MAG_CS  14
#define ACC_CS  15
#define BAR_CS  12

// ===================== REGISTER DEFINES =====================
#define REG_WHO_AM_I       0x00
#define REG_REVISION_ID    0x01
#define REG_CTRL1          0x02
#define REG_CTRL2          0x03
#define REG_CTRL3          0x04
#define REG_CTRL5          0x06
#define REG_CTRL7          0x08
#define REG_CTRL9          0x0A
#define REG_STATUSINT      0x2D
#define REG_STATUS0        0x2E
#define REG_TEMP_L         0x33
#define REG_GZ_H           0x40
#define REG_RESET          0x60

// Scale factors for ±16g, ±2048dps
float accelScale = 1.0f / 2048.0f;
float gyroScale  = 1.0f / 16.0f;
float tempScale  = 1.0f / 256.0f;

SPISettings imuSPISettings(1000000, MSBFIRST, SPI_MODE0);

// ===================== GLOBALS =====================
double temp;
double batteryVoltage, currentDraw, powerDraw;

float imuTemp;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

bool inaReady = false;
bool imuReady = false;

uint32_t goodReads = 0;
uint32_t timeouts = 0;

Generic_LM75 temperatureSensor;
INA226 INA(0x40);

// ===================== SPI HELPERS =====================

void imuWriteReg(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg & 0x7F);
  SPI1.transfer(val);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
}

uint8_t imuReadReg(uint8_t reg) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg | 0x80);
  uint8_t val = SPI1.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
  return val;
}

// Clean burst read - ONLY reads data, no debug prints, no status checks inside
void imuBurstRead(uint8_t startReg, uint8_t *buf, uint8_t len) {
  SPI1.beginTransaction(imuSPISettings);
  digitalWrite(IMU_CS, LOW);
  delayMicroseconds(1);
  SPI1.transfer(startReg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = SPI1.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
}

// ===================== IMU INIT =====================
bool imuInit() {
  // Soft reset
  imuWriteReg(REG_RESET, 0xB0);
  delay(100);

  uint8_t whoami = imuReadReg(REG_WHO_AM_I);
  Serial.print("  WHO_AM_I = 0x");
  Serial.println(whoami, HEX);
  if (whoami != 0x05) {
    Serial.println("  WHO_AM_I mismatch!");
    return false;
  }

  uint8_t rev = imuReadReg(REG_REVISION_ID);
  Serial.print("  Revision = 0x");
  Serial.println(rev, HEX);

  // CTRL1: auto-increment ON, little-endian, 4-wire SPI, clock on
  imuWriteReg(REG_CTRL1, 0x40);  // Just ADDR_AI=1, BE=0, SensorDisable=0
  delay(2);

  // Verify
  Serial.print("  CTRL1 readback = 0x");
  Serial.println(imuReadReg(REG_CTRL1), HEX);

  // CTRL2: accel ±16g, ODR=448.4Hz (aFS=011, aODR=0100)
  imuWriteReg(REG_CTRL2, 0x34);
  delay(2);

  // CTRL3: gyro ±2048dps, ODR=448.4Hz (gFS=111, gODR=0100)
  imuWriteReg(REG_CTRL3, 0x74);
  delay(2);

  // CTRL7: enable accel + gyro, NO SyncSample
  // bit7=0 (SyncSample off), bit1=1 (gEN), bit0=1 (aEN)
  imuWriteReg(REG_CTRL7, 0x03);
  delay(200);  // Wait for gyro startup (~150ms + settling)

  // Verify all config
  uint8_t c1 = imuReadReg(REG_CTRL1);
  uint8_t c2 = imuReadReg(REG_CTRL2);
  uint8_t c3 = imuReadReg(REG_CTRL3);
  uint8_t c7 = imuReadReg(REG_CTRL7);

  Serial.print("  CTRL1=0x"); Serial.print(c1, HEX);
  Serial.print("  CTRL2=0x"); Serial.print(c2, HEX);
  Serial.print("  CTRL3=0x"); Serial.print(c3, HEX);
  Serial.print("  CTRL7=0x"); Serial.println(c7, HEX);

  // Read STATUS0 once to clear any stale flags
  imuReadReg(REG_STATUS0);

  return true;
}

// ===================== IMU READ =====================
/*
  Non-SyncSample mode, simple approach:
  1. Read STATUS0 (0x2E) - check if aDA (bit0) or gDA (bit1) are set
  2. If data ready, burst read 14 bytes from TEMP_L (0x33)
  3. Reading STATUS0 clears the data-ready flags automatically
*/
bool imuReadAll() {
  // Check if new data is available
  uint8_t status = imuReadReg(REG_STATUS0);

  // We want both accel and gyro data ready (bits 0 and 1)
  if ((status & 0x03) == 0x00) {
    return false;  // No new data yet
  }

  // Burst read 14 bytes: temp(2) + accel(6) + gyro(6)
  uint8_t buf[14];
  imuBurstRead(REG_TEMP_L, buf, 14);

  // Parse little-endian two's complement
  int16_t rawTemp = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawAx   = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawAy   = (int16_t)((buf[5] << 8) | buf[4]);
  int16_t rawAz   = (int16_t)((buf[7] << 8) | buf[6]);
  int16_t rawGx   = (int16_t)((buf[9] << 8) | buf[8]);
  int16_t rawGy   = (int16_t)((buf[11] << 8) | buf[10]);
  int16_t rawGz   = (int16_t)((buf[13] << 8) | buf[12]);

  imuTemp = (float)rawTemp * tempScale;
  accelX  = (float)rawAx * accelScale;
  accelY  = (float)rawAy * accelScale;
  accelZ  = (float)rawAz * accelScale;
  gyroX   = (float)rawGx * gyroScale;
  gyroY   = (float)rawGy * gyroScale;
  gyroZ   = (float)rawGz * gyroScale;

  goodReads++;
  return true;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println("========================================");
  Serial.println("  AFS IMU TEST - Non-SyncSample Mode");
  Serial.println("========================================");

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

  int csPins[] = {IMU_CS, MAG_CS, ACC_CS, BAR_CS};
  for (int p : csPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  SPI1.setSCK(SPI_SCK);
  SPI1.setTX(SPI_SDO);
  SPI1.setRX(SPI_SDI);
  SPI1.begin();

  // INA226
  Serial.print("[INA226]   ");
  if (!INA.begin()) {
    Serial.println("FAIL");
  } else {
    INA.setMaxCurrentShunt(15, 0.002);
    inaReady = true;
    Serial.println("OK");
  }

  // IMU
  Serial.println("[IMU] Initializing...");
  if (!imuInit()) {
    Serial.println("[IMU] FAIL");
  } else {
    imuReady = true;
    Serial.println("[IMU] OK");
  }

  // Register dump
  Serial.println("\n  Register dump (0x00-0x0A):");
  for (uint8_t reg = 0x00; reg <= 0x0A; reg++) {
    uint8_t val = imuReadReg(reg);
    Serial.print("    0x");
    if (reg < 0x10) Serial.print("0");
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    if (val < 0x10) Serial.print("0");
    Serial.println(val, HEX);
  }

  // Also dump status registers
  Serial.print("    STATUSINT(0x2D) = 0x");
  Serial.println(imuReadReg(REG_STATUSINT), HEX);
  Serial.print("    STATUS0(0x2E)   = 0x");
  Serial.println(imuReadReg(REG_STATUS0), HEX);

  Serial.println("\n========================================");
  Serial.println("  Starting loop...");
  Serial.println("========================================\n");

  delay(500);
}

// ===================== LOOP =====================
void loop() {
  // LM75
  temp = temperatureSensor.readTemperatureC();

  // INA226
  if (inaReady) {
    batteryVoltage = INA.getBusVoltage();
    currentDraw    = INA.getCurrent_mA();
    powerDraw      = INA.getPower_mW();
  }

  // IMU - try to read, track success/fail
  bool gotData = false;
  if (imuReady) {
    // Try a few times in case we're between samples
    for (int attempt = 0; attempt < 10; attempt++) {
      if (imuReadAll()) {
        gotData = true;
        break;
      }
      delayMicroseconds(500);  // ~0.5ms between attempts
    }
    if (!gotData) {
      timeouts++;
    }
  }

  // Print
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

    Serial.print("  [reads=");
    Serial.print(goodReads);
    Serial.print("  timeouts=");
    Serial.print(timeouts);
    if (gotData) {
      Serial.println("  status=OK]");
    } else {
      Serial.print("  status=NO_DATA  STATUS0=0x");
      Serial.println(imuReadReg(REG_STATUS0), HEX);
    }
  } else {
    Serial.println("  IMU:          [offline]");
  }

  Serial.println("----------------------------------------\n");

  delay(200);
}
