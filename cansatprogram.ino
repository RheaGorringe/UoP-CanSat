/*
  SD card read/write

  This example shows how to read and write data to and from an SD card file
  The circuit. Pin numbers reflect the default
  SPI pins for Uno and Nano models:
   SD card attached to SPI bus as follows:
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (For For Uno, Nano: pin 10. For MKR Zero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified  24 July 2020
  by Tom Igoe

  This example code is in the public domain.

*/
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1023.1)

//Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

int cont = 0;
int Time = 0;

const int chipSelect = 52;
File myFile;

void setup() {
  // Open serial communications and wait for port to open:
    Serial.begin(9600);
//  // wait for Serial Monitor to connect. Needed for native USB port boards only:
//  while (!Serial);
//
//  Serial.print("Initializing SD card...");
//
  if (!SD.begin(chipSelect)) {
//    //Serial.println("initialization failed. Things to check:");
//    //Serial.println("1. is a card inserted?");
//    //Serial.println("2. is your wiring correct?");
//    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
//    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  
  }

  if (!bme.begin()) {
    //Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
 
  delay(2000);
  
    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (! bme.performReading()) {
    //Serial.println("Failed to perform reading :(");
    return;
  }
  delay(2000);
  
  //Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
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

//  myFile.print("Time (ms)");
//  myFile.print(",");
//  myFile.print("Temperature");
//  myFile.print(",");
//  myFile.print("Pressure");
//  myFile.print(",");
//  myFile.print("Altitude");

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");

  while (cont < 60){
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
      //Serial.print(Time);
      delay(200);
      Time += 200;
      // close the file:
      cont+=1;
  }
    myFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
  }

//  // re-open the file for reading:
//  myFile = SD.open("test.txt");
//  if (myFile) {
//    Serial.println("test.txt:");
//
//    // read from the file until there's nothing else in it:
//    while (myFile.available()) {
//      Serial.write(myFile.read());
//    }
//    // close the file:
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
}

void loop() {
  // nothing happens after setup
}
