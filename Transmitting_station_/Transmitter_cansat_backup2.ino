// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SD.h>

#define BME_SCK 10
#define BME_MISO 11
#define BME_MOSI 12
#define BME_CS 13
//pinout for BME

#define RFM95_CS 51
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 433

#define SEALEVELPRESSURE_HPA (1023.56)

#include <bme68x.h>
#include <bme68x_defs.h>

#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  // Feather M0 w/Radio
  #define RFM95_CS      8
  #define RFM95_INT     3
  #define RFM95_RST     4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_INT     9  // "A"
  #define RFM95_CS      10  // "B"
  #define RFM95_RST     11  // "C"
  
#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(ARDUINO_NRF52832_FEATHER)
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
File Myfile;

const int chipSelect = 52;
//for the sd card I think?
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool gbSenserConnectState = false;
 float convert = 1670.1;
 int16_t convertA = 2*9.81;
 int16_t convertG = 32.8;
 int16_t convertM = 0.15;
 

 //above - intialisation params for the accerometer sensor



Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);



void setup() 
{ 
  Serial.begin(115200);
  if (!SD.begin(chipSelect)) {
    //left here if debugging wanted later-
    Serial.println("problem with SD connection!");
    //while (true);
    //maybe make this if statement loop until the card is connected?
  }

    while (!Serial);
    
  ///initialise BME intitial params
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  
  if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    Serial.println("Pressure sersor NULL");
  }
  bme.setTemperatureOversampling(BME680_OS_8X); //(These are oversammpling/filter init)
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  
  
  //initialise IMU intiial params
  
  bool bRet;
  bool gbSenserConnectState = false;
  float convert = 1670.1;
  imuInit(&enMotionSensorType, &enPressureType);
  if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    Serial.println("Motion sersor NULL");
  }
  




  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  





  
}

//define a function that allows us to transmit/recieve data
int t = 0;

void Transmit(String data, int col) {

  int16_t packetnum = 0;  // packet counter, we increment per xmission
  data = String(t) + String(",") + String(col) + String(":") + data;
  int n = data.length();
  char radiopacket[n + 1];
  strcpy(radiopacket, data.c_str());

  //t, col
  
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  rf95.send((uint8_t *)radiopacket, 20);

  //rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
//  Serial.println("NOT Waiting for reply...");
//  if (rf95.waitAvailableTimeout(500))
//  { 
//    // Should be a reply message for us now   
//    if (rf95.recv(buf, &len))
//   {
//      Serial.print("Got reply: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
//    }
//    else
//    {
//      Serial.println("Receive failed");
//    }
////  }
//  else
//  {
//    Serial.println("No reply, is there a listener around?");
//  }







    //Serial.print("Done!");
  }




//  GLOBAL T VALUE DEFINED


void loop()
{
  
  

    
    //IMU data collection
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, 0);
//remov certain data that we don't use.

  bme.performReading();
  Serial.println("lAG");
  

  Transmit(String(millis()), 0);
  Transmit(String(bme.pressure), 1);
  Transmit(String(bme.temperature), 2);
  Transmit(String(bme.humidity), 3);
  Transmit(String(stAccelRawData.s16X / 1670.1), 4);
  Transmit(String(stAccelRawData.s16Y / 1670.1), 5);  
  Transmit(String(stAccelRawData.s16Z / 1670.1), 6);
  Transmit(String(stAngles.fRoll), 7);  
  Transmit(String(stAngles.fPitch), 8);  
  Transmit(String(stAngles.fYaw), 9);   
  Myfile = SD.open("backup.csv", FILE_WRITE);
  Myfile.print(String(millis()));
  Myfile.print(",");
  Myfile.print(String(bme.pressure));
  Myfile.print(",");
  Myfile.print(String(bme.temperature));
  Myfile.print(",");
  Myfile.print(String(bme.humidity));
  Myfile.print(",");
  Myfile.print(String(stAccelRawData.s16X / 1670.1));
  Myfile.print(",");
  Myfile.print(String(stAccelRawData.s16Y / 1670.1)); 
  Myfile.print(","); 
  Myfile.print(String(stAccelRawData.s16Z / 1670.1));
  Myfile.print(",");
  Myfile.print(String(stAngles.fRoll));  
  Myfile.print(",");
  Myfile.print(String(stAngles.fPitch));
  Myfile.print(",");  
  Myfile.println(String(stAngles.fYaw)); 
  Myfile.close();
  t++;


}
