#include "Waveshare_10Dof-D.h"
#include <bme68x.h>
#include <bme68x_defs.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

#include <RH_RF95.h>
//for rf transmitter/reciever

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1023.1)
//pinout for BME

#define RFM95_CS 50
#define RFM95_RST 2
#define RFM95_INT 3
//pinout for RF

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// Singleton instance of the radio driver



bool gbSenserConnectState = false;
 int16_t convertA = 2*9.81;
 int16_t convertG = 32.8;
 int16_t convertM = 0.15;

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

int cont = 0;
int Time = 0;


void setup() {
  // put your setup code here, to run once:
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  //wait for Serial Monitor to connect. Needed for native USB port boards only:
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(115200);

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
  int16_t packetnum = 0;  // packet counter, we increment per xmission


  

  delay(1000);
  !bme.begin();
  imuInit(&enMotionSensorType, &enPressureType);

  
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

 
}
int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  // put your main code here, to run repeatedly:
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
  
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
  }

  

  delay(100); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println(bme.temperature); // Send a message to rf95_server
  String pressure = String(bme.pressure);
  char radiopacket[6] = {pressure[0], pressure[1], pressure[2], pressure[3], pressure[4], pressure[5]};
  //itoa(packetnum++, radiopacket, 10);
  Serial.print("Sending pressure:  "); Serial.println(radiopacket);
  uint8_t radio[6] = {radiopacket[0], radiopacket[1], radiopacket[2], radiopacket[3], radiopacket[4], radiopacket[5]};
  //radiopacket[19] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t*)radio, 6);
  
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }

  
//
//while (cont < 10){
//      myFile.print("\n");
//      myFile.print(Time);
//      myFile.print(",");
//      myFile.print(bme.temperature);
//      myFile.print(",");
//      myFile.print(bme.pressure);
//      myFile.print(",");
//      myFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//      myFile.print(",");
//      myFile.print(bme.humidity);
//      myFile.print(",");
//      myFile.print(bme.gas_resistance /1000);
//      Serial.println(stAngles.fRoll);
//      myFile.print(",");
//      Serial.println(stAngles.fPitch);
//      myFile.print(",");
//      Serial.println(stAngles.fYaw);
//      myFile.print(",");
//      Serial.println(stAccelRawData.s16X / convertA);
//      myFile.print(",");
//      Serial.println(stAccelRawData.s16Y / convertA);
//      myFile.print(",");
//      Serial.println(stAccelRawData.s16Z / convertA);
//      myFile.print(",");
//      Serial.println(stGyroRawData.s16X / 32.8);
//      myFile.print(",");
//      Serial.println(stGyroRawData.s16Y / 32.8);
//      myFile.print(",");
//      Serial.println(stGyroRawData.s16Z / 32.8);
//      myFile.print(",");
//      Serial.println(stMagnRawData.s16X *0.15);
//      myFile.print(",");
//      Serial.println(stMagnRawData.s16Y *0.15);
//      myFile.print(",");
//      Serial.println(stMagnRawData.s16Z *0.15);
//
//      delay(200);
//      Time += 200;
//      // close the file:
//      cont+=1;
  }
