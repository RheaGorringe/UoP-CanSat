#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <stdlib.h>
#include <string.h>
#include <string> 

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3  


#define RF95_FREQ 433.0
#define LED 13
RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
  #define LED           17
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif
const int chipSelect = 9;
int red_light_pin= 5;
int green_light_pin = 6;
int blue_light_pin = 7;

//ABOVE CAN BE CHANged for jusT THE DUE

File Myfile;

void setup() {
  Serial.begin(115200);


  
  //setting up reciever Led
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);  
  
  
  if (!SD.begin(chipSelect)) {
  Serial.println("initialization failed. Things to check:");
  Serial.println("1. is a card inserted?");
  Serial.println("2. is your wiring correct?");
  Serial.println("3. did you change the chipSelect pin to match your shield or module?");
  Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  }
  else{Serial.println("SUCCESSFUL SD INIT");}
   
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  while (!Serial) {
    delay(1);
  }
  delay(100);

  Serial.println("Feather LoRa RX Test!");

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

void RGB_color(int red_light_value , int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
} 

//UNIVERSAL T
int t = 0;
//Equates to row
String rec;
int T = 0;
//Equates to col



void loop() {
 // put your main code here, to run repeatedly:

  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      Serial.println((char*)buf);
      //use for changing col of led.

//      // Send a reply
//      uint8_t data[] = "And hello back to you";
//      rf95.send(data, sizeof(data));
//      rf95.waitPacketSent();
//      Serial.println("Sent a reply");
//      digitalWrite(LED, LOW);

      char *s;
      int i;

      char *S;
      int I;

      String row;
      String col;
      String dat;
      //string so that the characters recieved can be concactenated
      //data

      char* b = (char*)buf;
      s = strstr(b, ",");
      i = s - b;

      S = strstr(b, ":");
      I = S - b;

      
      //int c = sizeof(b);
      ///Used later. can prob be optimised by putting straight into col loop
      
      //for some reason this gives the index where the separator is found! (To do with memory addresse correlating to elements in arrays somehow).
      for (int j = 0; j < (i); ++j) {
        row+=(String(b[j]));
    
      }
      for (int j = i+1; j < (I); ++j) {
        col+=(String(b[j]));
        
    
      }
      for (int j = I+1; j < (String(b).length()); ++j) {
        //to optimise, maybe the size function works on char*s?
        //for some reason, b has is a long value BUT
        //is said to have a length of just 4??? due to pointers and stuff..
        dat+=(String(b[j]));
    
      }


      int COL = (int(char(col[0])) - 48);
//      Debugging has sucked the soul out of me so I think I'll leave this temp fix in for now.
//     I thought there was a problem with data types, when it turns out if statements with an "or" in them did not work with what I was
//      comparing..
            
      if(COL != T){
        Serial.println("WRONG COL");
        Serial.println(T);
        Serial.println(COL);
        T = 0;

      }
      
      if(row.toInt() != t){
        Serial.println("WRONG ROW");
        Serial.println(t);
        Serial.println(row.toInt());
        t = row.toInt() + 1;    
      }


      //the two above statements

      else{
      //whereas here we will set the boolean CHECK to false as no data check needed.

      



    
    
    


    
      //this needs to be changed when increasing/decrease data in
      
      //since column "1" will be index 0 

      if (T == 9){
        T=0;
        //Serial.println(rec);
          //this too
        rec+=dat;
        Serial.println("rec");
        Serial.println(rec.length());
        Serial.println("Data");
        Serial.println(rec);
        Myfile = SD.open("data.csv", FILE_WRITE);
        Myfile.print(t);
        //writes in a "time" value. Placeholder for real time
        Myfile.print(",");
        Myfile.println(rec);
        Myfile.close();
        Myfile.print("WRITTEN!");
        t++;

        rec = "";
      }
      else{ 
        rec+=dat; 
        rec+=","; 
        Serial.println(rec);
        T++;
        //next column
      }
      //this will change to 2 for when this loop will only record pressure
      //another loop will do the other thing.
  
  
      Serial.println(rf95.lastRssi());
      int P = rf95.lastRssi();
      
      if ( -50 < P && P < 0){
        
        RGB_color(0      ,     (-P)        ,    (P) + 100      ); 
        }
      if ( -100  <  P && P  < -50){
        
        RGB_color(                 0, (-P)    ,         (P) + 100); 
        }
      if (-130 < rf95.lastRssi() && rf95.lastRssi() < -90.0){
        
        RGB_color(      4*(-P - 90),          (100 - 2*(-P - 90)  )  ,                   0  );
        }
       //determines colour of LEDS based on signal strength    
      
      delay(10);
      
  
  
      
  
        }    
      }
      
    else{
     Serial.println("Receive failed");
     RGB_color(1,1,1);
    }
   //myFile = SD.open("data.csv", FILE_WRITE);
   //myFile.close();
  
      }
  
  
 }
 //else loop for checking t
