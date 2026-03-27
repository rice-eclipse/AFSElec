/*
  RICE ECLIPSE AFS - IMU Bare Minimum Test
  No auto-increment. No SyncSample. Individual register reads only.
*/

#include <Temperature_LM75_Derived.h>
#include <INA226.h>
#include <Wire.h>
#include <SPI.h>

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

SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE0);

double temp;
double batteryVoltage, currentDraw, powerDraw;
float imuTemp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
bool inaReady = false;
bool imuReady = false;
uint32_t readCount = 0;

Generic_LM75 temperatureSensor;
INA226 INA(0x40);

// --- Single register write ---
void imuWrite(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(imuSPI);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg & 0x7F);
  SPI1.transfer(val);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
}

// --- Single register read ---
uint8_t imuRead(uint8_t reg) {
  SPI1.beginTransaction(imuSPI);
  digitalWrite(IMU_CS, LOW);
  SPI1.transfer(reg | 0x80);
  uint8_t val = SPI1.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  SPI1.endTransaction();
  return val;
}

bool imuInit() {
  // Soft reset
  imuWrite(0x60, 0xB0);
  delay(100);

  // Check WHO_AM_I
  uint8_t id = imuRead(0x00);
  Serial.print("  WHO_AM_I=0x"); Serial.println(id, HEX);
  if (id != 0x05) return false;

  // CTRL1: no auto-increment, little-endian, 4-wire SPI, clock on
  // bit7=0(4wire) bit6=0(no AI) bit5=0(LE) bit0=0(clock on)
  imuWrite(0x02, 0x00);
  delay(2);

  // CTRL2: accel ±16g, ODR 448.4Hz => aFS=011, aODR=0100 => 0x34
  imuWrite(0x03, 0x34);
  delay(2);

  // CTRL3: gyro ±2048dps, ODR 448.4Hz => gFS=111, gODR=0100 => 0x74
  imuWrite(0x04, 0x74);
  delay(2);

  // Disable AHB clock gating (needed for reliable data reads)
  // Write 0x01 to CAL1_L (0x0B), then send CTRL9 command 0x12
  imuWrite(0x0B, 0x01);
  delay(1);
  imuWrite(0x0A, 0x12);  // CTRL_CMD_AHB_CLOCK_GATING
  delay(50);
  // Acknowledge: write 0x00 to CTRL9
  imuWrite(0x0A, 0x00);
  delay(2);

  // CTRL7: accel+gyro on, NO SyncSample
  // bit7=0, bit1=1(gEN), bit0=1(aEN) => 0x03
  imuWrite(0x08, 0x03);
  delay(200);

  // Verify
  Serial.print("  CTRL1=0x"); Serial.println(imuRead(0x02), HEX);
  Serial.print("  CTRL2=0x"); Serial.println(imuRead(0x03), HEX);
  Serial.print("  CTRL3=0x"); Serial.println(imuRead(0x04), HEX);
  Serial.print("  CTRL7=0x"); Serial.println(imuRead(0x08), HEX);

  // Clear stale status
  imuRead(0x2E);

  return true;
}

void imuReadAll() {
  // Read each register individually, no auto-increment, no burst
  uint8_t tl = imuRead(0x33);
  uint8_t th = imuRead(0x34);
  uint8_t axl = imuRead(0x35);
  uint8_t axh = imuRead(0x36);
  uint8_t ayl = imuRead(0x37);
  uint8_t ayh = imuRead(0x38);
  uint8_t azl = imuRead(0x39);
  uint8_t azh = imuRead(0x3A);
  uint8_t gxl = imuRead(0x3B);
  uint8_t gxh = imuRead(0x3C);
  uint8_t gyl = imuRead(0x3D);
  uint8_t gyh = imuRead(0x3E);
  uint8_t gzl = imuRead(0x3F);
  uint8_t gzh = imuRead(0x40);

  int16_t rawT  = (int16_t)((th  << 8) | tl);
  int16_t rawAx = (int16_t)((axh << 8) | axl);
  int16_t rawAy = (int16_t)((ayh << 8) | ayl);
  int16_t rawAz = (int16_t)((azh << 8) | azl);
  int16_t rawGx = (int16_t)((gxh << 8) | gxl);
  int16_t rawGy = (int16_t)((gyh << 8) | gyl);
  int16_t rawGz = (int16_t)((gzh << 8) | gzl);

  imuTemp = (float)rawT  / 256.0f;
  accelX  = (float)rawAx / 2048.0f;
  accelY  = (float)rawAy / 2048.0f;
  accelZ  = (float)rawAz / 2048.0f;
  gyroX   = (float)rawGx / 16.0f;
  gyroY   = (float)rawGy / 16.0f;
  gyroZ   = (float)rawGz / 16.0f;

  readCount++;
}

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println("==== AFS IMU TEST (no AI, no sync) ====");

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

  int cs[] = {IMU_CS, MAG_CS, ACC_CS, BAR_CS};
  for (int p : cs) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  SPI1.setSCK(SPI_SCK);
  SPI1.setTX(SPI_SDO);
  SPI1.setRX(SPI_SDI);
  SPI1.begin();

  Serial.print("[INA226] ");
  if (!INA.begin()) { Serial.println("FAIL"); }
  else { INA.setMaxCurrentShunt(15, 0.002); inaReady = true; Serial.println("OK"); }

  Serial.println("[IMU] Init...");
  if (!imuInit()) { Serial.println("[IMU] FAIL"); }
  else { imuReady = true; Serial.println("[IMU] OK"); }

  Serial.println("==== Loop starting ====\n");
  delay(300);
}

void loop() {
  temp = temperatureSensor.readTemperatureC();

  if (inaReady) {
    batteryVoltage = INA.getBusVoltage();
    currentDraw = INA.getCurrent_mA();
    powerDraw = INA.getPower_mW();
  }

  if (imuReady) {
    imuReadAll();
  }

  Serial.println("----------------------------------------");
  Serial.print("  Board Temp: "); Serial.print(temp, 1); Serial.println(" C");

  if (inaReady) {
    Serial.print("  V:"); Serial.print(batteryVoltage, 3);
    Serial.print("  I:"); Serial.print(currentDraw, 3);
    Serial.print("mA  P:"); Serial.print(powerDraw, 3); Serial.println("mW");
  }

  if (imuReady) {
    Serial.print("  IMU T: "); Serial.print(imuTemp, 2); Serial.println(" C");
    Serial.print("  Acc  X:"); Serial.print(accelX, 3);
    Serial.print(" Y:"); Serial.print(accelY, 3);
    Serial.print(" Z:"); Serial.println(accelZ, 3);
    Serial.print("  Gyro X:"); Serial.print(gyroX, 2);
    Serial.print(" Y:"); Serial.print(gyroY, 2);
    Serial.print(" Z:"); Serial.println(gyroZ, 2);
    Serial.print("  [#"); Serial.print(readCount); Serial.println("]");
  }

  Serial.println();
  delay(200);
}
