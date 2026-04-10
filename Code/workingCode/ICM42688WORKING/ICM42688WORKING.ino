/*
  RICE ECLIPSE AFS - IMU FIFO Build
  Features: CTRL9 Handshake + FIFO Register Mapping
*/

#include <Temperature_LM75_Derived.h>
#include <INA226.h>
#include <Wire.h>
#include <SPI.h>

// ===================== PIN DEFINITIONS =====================
#define led1 25
#define led2 5
#define I2C_SDA 20
#define I2C_SCL 21
#define SPI_SDO 11
#define SPI_SDI 8
#define SPI_SCK 10
#define IMU_CS  13
#define MAG_CS  14
#define ACC_CS  15
#define BAR_CS  12

// ===================== REGISTER MAP =====================
#define REG_WHO_AM_I       0x00
#define REG_CTRL1          0x02
#define REG_CTRL2          0x03
#define REG_CTRL3          0x04
#define REG_CTRL7          0x08
#define REG_CTRL9          0x0A  // Host command register
#define REG_FIFO_CTRL      0x14  // FIFO Control (Release)
#define REG_FIFO_SMPL_L    0x15  // FIFO sample count LSB
#define REG_FIFO_STATUS    0x16  // FIFO sample count MSB
#define REG_FIFO_DATA      0x17  // FIFO data port
#define REG_STATUSINT      0x2D
#define REG_TEMP_L         0x33
#define REG_TEMP_H         0x34
#define REG_RESET          0x60


// ===================== GLOBALS =====================
float accelScale = 1.0f / 2048.0f;
float gyroScale  = 1.0f / 16.0f;
float tempScale  = 1.0f / 256.0f;

SPISettings imuSPISettings(1000000, MSBFIRST, SPI_MODE0);

double temp, batteryVoltage, currentDraw, powerDraw;
float imuTemp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
bool inaReady = false, imuReady = false;

Generic_LM75 temperatureSensor;
INA226 INA(0x40);

// ===================== HELPERS =====================

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

// ===================== IMU LOGIC =====================

bool imuInit() {
  imuWriteReg(REG_RESET, 0xB0);//reset sensor
  delay(100);

  if (imuReadReg(REG_WHO_AM_I) != 0x05) return false; // check for proper initialization

  imuWriteReg(REG_CTRL1, 0x60); //auto increment on, MSB on
  imuWriteReg(REG_CTRL2, 0x34); // ±16g - accelerometer settings
  imuWriteReg(REG_CTRL3, 0x74); // ±2048dps - gyroscope settings
  
  // Initialize FIFO in Stream Mode (0x02) to stream data straight to registers, without saving in buffer
  imuWriteReg(REG_FIFO_CTRL, 0x02); 
  
  imuWriteReg(REG_CTRL7, 0x03); // Enable sensors
  delay(50);
  return true;
}

void imuReadAll() {
  // 1. Check if samples are available in FIFO
  uint8_t countL = imuReadReg(REG_FIFO_SMPL_L);
  uint8_t status = imuReadReg(REG_FIFO_STATUS);
  uint16_t samplesAvail = ((status & 0x03) << 8) | countL;

  if (samplesAvail > 0) {
    // 2. CTRL9 Handshake: Enable FIFO Read Mode
    imuWriteReg(REG_CTRL9, 0x05); 

    // 3. Burst Read Data from registers since we are in Stream mode, normal registers.
    uint8_t buf[12];
    SPI1.beginTransaction(imuSPISettings);
    digitalWrite(IMU_CS, LOW);
    
    SPI1.transfer(0x35 | 0x80);//start of burst read
    for (int i = 0; i < 12; i++) {
      buf[i] = SPI1.transfer(0x00); // next sensor register in burst read
    }
    digitalWrite(IMU_CS, HIGH);
    SPI1.endTransaction();

    // 4. Release FIFO: Reset FIFO_CTRL to clear RD_MODE (Bit 7)
    imuWriteReg(REG_FIFO_CTRL, 0x02); 

    // 5. Process Little-Endian Data
    accelX = (int16_t)((buf[1] << 8) | buf[0]) * accelScale;
    accelY = (int16_t)((buf[3] << 8) | buf[2]) * accelScale;
    accelZ = (int16_t)((buf[5] << 8) | buf[4]) * accelScale;
    gyroX  = (int16_t)((buf[7] << 8) | buf[6]) * gyroScale;
    gyroY  = (int16_t)((buf[9] << 8) | buf[8]) * gyroScale;
    gyroZ  = (int16_t)((buf[11] << 8) | buf[10]) * gyroScale;

    // Read Temperature separately (non-FIFO register)
    int16_t rawTemp = (int16_t)((imuReadReg(REG_TEMP_H) << 8) | imuReadReg(REG_TEMP_L));
    imuTemp = (float)rawTemp * tempScale;
  }
}

// ===================== STANDARD SETUP/LOOP =====================
void setup() {
  Serial.begin(115200);
  //make sure no sensor interferance by disabling pins
  int csPins[] = {IMU_CS, MAG_CS, ACC_CS, BAR_CS};
  for (int p : csPins) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }

  //setup i2c
  Wire.setSDA(I2C_SDA); 
  Wire.setSCL(I2C_SCL); 
  Wire.begin();

  //setup SPI
  SPI1.setSCK(SPI_SCK); 
  SPI1.setTX(SPI_SDO); 
  SPI1.setRX(SPI_SDI); 
  SPI1.begin();

  if (INA.begin()) { INA.setMaxCurrentShunt(15, 0.002); inaReady = true; }
  if (imuInit()) imuReady = true;
}

void loop() {
  // temp = temperatureSensor.readTemperatureC();
  // if (inaReady) {
  //   batteryVoltage = INA.getBusVoltage();
  //   currentDraw = INA.getCurrent_mA();
  // }
  if (imuReady) imuReadAll();

  // Serial.println("----------------------------------------");
  // Serial.print("Board: "); Serial.print(temp, 1); Serial.println(" C");
  if (imuReady) {
    // Serial.print("Accel: ");
    Serial.print(accelX, 2); Serial.print(", "); Serial.print(accelY, 2); Serial.print(", "); Serial.println(accelZ, 2);
  }
  // delay(200);
}