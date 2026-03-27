#include <Arduino.h>
#include <SPI.h>

// Pins
const int CS_ICM = 13;   // ICM-42688-PC imu
const int CS_MS56 = 12;  // MS5611 barometer
const int CS_KX = 15;    // KX134 high g accel
const int CS_MMC = 14;   // MMC5983MA magnetometer
const int PIN_SCK = 10;
const int PIN_MOSI = 11;
const int PIN_MISO = 8;

// Different "Languages" for the same bus
SPISettings settingsLSB(1000000, LSBFIRST, SPI_MODE0); 
SPISettings settingsMSB(1000000, MSBFIRST, SPI_MODE0); 

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(2000);

  // Initialize all CS pins to HIGH
  for(int p : {CS_ICM, CS_MS56, CS_KX, CS_MMC}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }

  SPI1.setSCK(PIN_SCK);
  SPI1.setTX(PIN_MOSI);
  SPI1.setRX(PIN_MISO);
  SPI1.begin();

  // RESET MS5611 (Must be MSB) - this helped stuff be more consistent, helps with bus contention i think
  // SPI1.beginTransaction(settingsMSB);
  // digitalWrite(CS_MS56, LOW);
  // SPI1.transfer(0x1E); 
  // digitalWrite(CS_MS56, HIGH);
  // SPI1.endTransaction();
  // delay(15); 
}

void loop() {
  Serial.println("--- SWEEP RESULTS ---");

  // 1. Read ICM (LSB Mode)
  SPI1.beginTransaction(settingsMSB);
  digitalWrite(CS_ICM, LOW);
  SPI1.transfer(0x00);              // Request Read @ 0x00    Transfer is simultaneous read/write, so you have to do the first transfer to tell the sensor what you want and the second to actually read it
  byte icm_id = SPI1.transfer(0x00); // Receive ID (Expected 0x05)
  digitalWrite(CS_ICM, HIGH);
  SPI1.endTransaction();

  // 2. Read MS5611 (MSB Mode)
  SPI1.beginTransaction(settingsMSB);
  digitalWrite(CS_MS56, LOW);
  SPI1.transfer(0xA2);               // Read Command for one of the calibration values
  byte high = SPI1.transfer(0x00); //16 bit read
  byte low  = SPI1.transfer(0x00);
  digitalWrite(CS_MS56, HIGH);
  int ms_prom = (high << 8) | low;
  SPI1.endTransaction();

  // 3. Read KX134 (MSB Mode)
  SPI1.beginTransaction(settingsMSB);
  digitalWrite(CS_KX, LOW);
  SPI1.transfer(0x13 | 0x80);        // WHO_AM_I Address, bitwise or with 0x80 because leading 1 indicates read - same for mmc5983
  byte kx_id = SPI1.transfer(0x00);  // Receive ID (Expected 0x46)
  digitalWrite(CS_KX, HIGH);
  SPI1.endTransaction();

  // 4. Read MMC5983MA (MSB Mode)
  // Product ID is in Register 0x2F
  SPI1.beginTransaction(settingsMSB);
  digitalWrite(CS_MMC, LOW);
  SPI1.transfer(0x2F | 0x80);        // Read @ 0x2F
  byte mmc_id = SPI1.transfer(0x00); // Receive ID (Expected 0x30)
  digitalWrite(CS_MMC, HIGH);
  SPI1.endTransaction();

  // Print Summary
  Serial.print("ICM-42688 ID (0x05): 0x"); Serial.println(icm_id, HEX);
  Serial.print("MS5611 PROM (non-0): 0x"); Serial.print(high, HEX);  Serial.println(low, HEX); 
  Serial.print("KX134 ID    (0x46): 0x"); Serial.println(kx_id, HEX);
  Serial.print("MMC5983 ID  (0x30): 0x"); Serial.println(mmc_id, HEX);

  delay(1000);
}