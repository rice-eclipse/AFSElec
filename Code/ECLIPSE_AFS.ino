/*
RICE ECLISPE AFS Basic Code


*/
//sensor libraries
#include <Temperature_LM75_Derived.h>
#include <INA226.h>

//other libraries
#include <Wire.h>

//pin definitions
#define led1 25 //yellow indicator led - can also use LED_BUILTIN instead of led1
#define led2 5 //green indicator led

#define current1 26//4 current detecting pins for servos
#define current2 27
#define current3 28
#define current4 29

#define PWM1 19 //pwm pins
#define PWM2 22
#define PWM3 23
#define PWM4 24

#define I2C_SDA 20 //I2C pins
#define I2C_SCL 21

#define SPI_SDO 8 //sensor SPI pins
#define SPI_SDI 11
#define SPI_SCK 10

#define IMU_CS 13 //6 axis IMU SPI select pin 
#define MAG_CS 14 //3 axis Magnetometer SPI select pin 
#define ACC_CS 15 //3 axis high g accelerometer SPI select pin 
#define BAR_CS 12 //barometer SPI select pin 

//whether to print things or not, printing will slow down program so turn off for real testing maybe?
int debug = true;

//define sensors
Generic_LM75 temperatureSensor;
INA226 INA(0x40);


void setup() {
  // initialize pin modes
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

  

  // Wait for Serial to initialize
  while(!Serial) {
    delay(10);
  }
  
  //begin serial monitor
  if (debug){
    Serial.begin(115200);
  }

  // Configure RP2350 pins for I2C1
  // GPIO 20 = SDA, GPIO 21 = SCL
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  //setup INA226
  if (!INA.begin() )
  {
    Serial.println("could not connect to INA226. Fix and Reboot");
  }
  INA.setMaxCurrentShunt(15, 0.002);
}

//storing of values
double temp;

double currDraw1;
double currDraw2;
double currDraw3;
double currDraw4;

double batteryVoltage;
double shuntVoltage;
double currentDraw;
double powerDraw;


void updateData(){
  temp = temperatureSensor.readTemperatureC();

  currDraw1 = map(analogRead(current1),0,1023,-5000,5000)/1000.0;
  currDraw2 = map(analogRead(current2),0,1023,-5000,5000)/1000.0;
  currDraw3 = map(analogRead(current3),0,1023,-5000,5000)/1000.0;
  currDraw4 = map(analogRead(current4),0,1023,-5000,5000)/1000.0;

  batteryVoltage = INA.getBusVoltage();
  shuntVoltage = INA.getShuntVoltage_mV();
  currentDraw = INA.getCurrent_mA();
  powerDraw = INA.getPower_mW();

}


void loop() {
  updateData();
  
  if (debug){
    Serial.print("Temperature = ");
    Serial.print(temp,1);
    Serial.print(" C, ");

    Serial.print("Power Draw = ");
    Serial.print(powerDraw,3);
    Serial.print(" mW, ");

    Serial.println();


  }
  
}

