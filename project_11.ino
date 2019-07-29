#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"

SoftwareSerial gps(6, 5);
SoftwareSerial DFPlayerSerial(2, 3);

TinyGPSPlus tinyGPS; // tinyGPSPlus object to be used throughout

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setup()
{
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(9600);

  delay(1000);
  //Serial.println("Setting up SD card.");
  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(4, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  if (!file.open("gpslog.txt", O_CREAT | O_WRITE)) {
    error("file.open");
  }
//  Serial.print("File name: ");
//  Serial.println("gpslog.txt"); // Debug print the file name

  
  DFPlayerSerial.begin (9600);
  gps.begin(9600);
  mp3_set_serial (DFPlayerSerial);
  delay(1);  //wait 1ms for mp3 module to set volume
  mp3_set_volume (0);
  freeRam ();
  delay(2000); Serial.flush();
}

uint32_t timer = millis();

void loop()                     // run over and over again
{
        // If we're not logging, continue to "feed" the tinyGPS object:
    while (gps.available())
      tinyGPS.encode(gps.read());
        
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,14,true);  // request a total of 14 registers
      int16_t AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      int16_t AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      int16_t AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      int16_t Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      int16_t GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      int16_t GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      int16_t GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

//    if (GPS.fix) {
      String StoServer = StoServer + "[0" + "," + String(tinyGPS.location.lat(), 6) + "," + String(tinyGPS.location.lng(), 6) + "," + String(tinyGPS.date.value()) + "," + String(tinyGPS.time.value()) + "," + AcX + "," + AcY + "," + AcZ + "," + String(Tmp/340.00+36.53) + "," + GyX + "," + GyY + "," + GyZ +"]";
      Serial.println(StoServer); Serial.flush();
      file.println(StoServer); file.flush();
      freeRam ();
      StoServer = "\0";
//    }
  }
  

  if(Serial.available()){
    String SfromServer = Serial.readStringUntil('\n');
    //Serial.println(SfromServer);
    StaticJsonBuffer<50> jsonBuffer;
    JsonArray& array = jsonBuffer.parseArray(SfromServer);
    SfromServer = "\0";  
  //  Serial.println(String(array));
    
    if(array.success()){
  //        fence = array[1];
          int state = array[0];
          int volume = array[1];
          int sound = array[2];   
          int flag = array[3];
  
          if(flag == 0 || flag == 1){ //flag 0 or 1  
            mp3_set_volume (volume);
            delay(10);
            playMusic_1(sound, state);
          }
          else {
            mp3_set_volume (5);
            delay(10);
            playMusic_3(sound, state);
          }
      Serial.flush();
      volume = 0;
      delay(10);
    }
  }
}

//flag 0 or 1
void playMusic_1(int sound, int state){
       if(sound == 1){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (3);
              break;

          }
     }
     else if(sound == 2){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (6);
              break;
          }
     }
     else if(sound == 3){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (9);
              break;
          }
     }
     else if(sound == 4){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (12);
              break;
          }
     }
     else if(sound == 5){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (15);
              break;
          }
     }
     else if(sound == 6){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (18);
              break;
          }
     }
     else if(sound == 7){
            switch (state) {
            case 0: //stop playing
              mp3_stop();
              break;

            default:
              mp3_play (21);
              break;
          }
     }
           else Serial.println("NO SOUND");
}

void playMusic_3(int sound, int state){
     if(sound == 1){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (1);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (2);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (3);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 2){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (4);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (5);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (6);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 3){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (7);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (8);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (9);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 4){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (10);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (11);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (12);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 5){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (13);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (14);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (15);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 6){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (16);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (17);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (18);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
     else if(sound == 7){
            switch (state) {
            case 1: //Low signal
              // mp3_set_volume (5);
              mp3_play (19);
              break;
            case 2: //Middle signal
              //  mp3_set_volume (15);
              mp3_play (20);
              break;
            case 3: //Hight signal
              // mp3_set_volume (25);
              mp3_play (21);
              break;
            case 0: //stop playing
              mp3_stop();
              break;
      
            default:
              //
              break;
          }
     }
           else Serial.println("NO SOUND");
}


