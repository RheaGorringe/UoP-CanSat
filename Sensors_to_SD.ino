#include "Waveshare_10Dof-D.h"
#include <bme68x.h>
#include <bme68x_defs.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1023.1)

bool gbSenserConnectState = false;
 int16_t convertA = 2*9.81;
 int16_t convertG = 32.8;
 int16_t convertM = 0.15;

 Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

int cont = 0;
int Time = 0;

const int chipSelect = 52;
File myFile;

void setup() {
  // put your setup code here, to run once:
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  Serial.begin(9600);
  //wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
      Serial.println("initialization failed. Things to check:");
      Serial.println("1. is a card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("3. did you change the chipSelect pin to match your shield or module?");
      Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  }
 !bme.begin();

 delay(1000);
  imuInit(&enMotionSensorType, &enPressureType);

 myFile = SD.open("test.csv", FILE_WRITE);
 
  myFile.print("Time (ms)");
  myFile.print(",");
  myFile.print("Temperature (C)");
  myFile.print(",");
  myFile.print("Pressure (hPa)");
  myFile.print(",");
  myFile.print("Altitude (m)");
  myFile.print(",");
  myFile.print("Humidity (%)");
  myFile.print(",");
  myFile.print("Gas resistance (KOhms)");
  myFile.print(",");
  myFile.print("Roll : "); 
  myFile.print(",");
  myFile.print("    Pitch : "); 
  myFile.print(",");
  myFile.print("    Yaw : "); 
  myFile.print(",");
  myFile.print("Acceleration: X : "); 
  myFile.print(",");
  myFile.print("    Acceleration: Y : "); 
  myFile.print(",");
  myFile.print("    Acceleration: Z : "); 
  myFile.print(",");
  myFile.print("Gyroscope: X : "); 
  myFile.print(",");
  myFile.print("       Gyroscope: Y : "); 
  myFile.print(",");
  myFile.print("       Gyroscope: Z : "); 
  myFile.print(",");
  myFile.print("Magnetic: X : "); 
  myFile.print(",");
  myFile.print("      Magnetic: Y : "); 
  myFile.print(",");
  myFile.print("      Magnetic: Z : ");
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
  
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  //pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

while (cont < 300){
      myFile.print("\n");
      myFile.print(Time);
      myFile.print(",");
      myFile.print(bme.temperature);
      myFile.print(",");
      myFile.print(bme.pressure);
      myFile.print(",");
      myFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      myFile.print(",");
      myFile.print(bme.humidity);
      myFile.print(",");
      myFile.print(bme.gas_resistance /1000);
      myFile.print(stAngles.fRoll);
      myFile.print(",");
      myFile.print(stAngles.fPitch);
      myFile.print(",");
      myFile.print(stAngles.fYaw);
      myFile.print(",");
      myFile.print(stAccelRawData.s16X / convertA);
      myFile.print(",");
      myFile.print(stAccelRawData.s16Y / convertA);
      myFile.print(",");
      myFile.print(stAccelRawData.s16Z / convertA);
      myFile.print(",");
      myFile.print(stGyroRawData.s16X / 32.8);
      myFile.print(",");
      myFile.print(stGyroRawData.s16Y / 32.8);
      myFile.print(",");
      myFile.print(stGyroRawData.s16Z / 32.8);
      myFile.print(",");
      myFile.print(stMagnRawData.s16X *0.15);
      myFile.print(",");
      myFile.print(stMagnRawData.s16Y *0.15);
      myFile.print(",");
      myFile.print(stMagnRawData.s16Z *0.15);

      delay(200);
      Time += 200;
      // close the file:
      cont+=1;
  }
    myFile.close();
    Serial.println("done.");
}
