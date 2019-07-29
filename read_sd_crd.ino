#include <SoftwareSerial.h>
#include <virtuabotixRTC.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <stdlib.h>
#include "SdFat.h"
SdFat SD;
#include "Wire.h" // This library allows you to communicate with I2C devices.

#define mpu_add 0x68
#define sleep_acy 10000
#define PIN_CLK 7
#define PIN_DAT 6
#define PIN_RST 5
int RedPin = A0; //the Red pin connects to digital pin 11 on the arduino
int BluePin = A1; //the Blue pin connects to digital pin 10 on the arduino
int GreenPin = A2 ;//the Green pin connects to digital pin 11 on the arduino
int healthGood = A3;
int healthNorm = 8;
unsigned long interval=1000; // the time we need to wait
unsigned long previousMillis=0; // millis() returns an unsigned long.
int chipSelect = 9;
long ac_x, ac_y,ac_z,gy_x,gy_y,gy_z ; //acc, gyro data 
long i=0, Maxgy,Mingy,Gapgy,sleep;
long gyy[8], acy[8];
int BTpin = 8;

//const int chipSelect = 10;
//detect time for time and date
virtuabotixRTC my_rtc(PIN_CLK,PIN_DAT,PIN_RST);
// call Sd card fuction
#define SD_CS_PIN SS
File myFile;

SoftwareSerial mySerial(3, 4); // RX, TX
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
char numChars = 32;
//char receivedChars[numChars];
int sec;
 
void setup() 
{
   // set the BTpin for input
    pinMode(BTpin, INPUT);   
 
  // HC-06 default serial speed for communcation mode is 9600
    mySerial.begin(9600);
    Serial.begin(9600);
    // data hand set
    //my_rtc.setDS1302Time(0,50,14,9,13,11,2018);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
      // see if the card is present and can be initialized:
     pinMode(chipSelect, OUTPUT);
    if (!SD.begin(SD_CS_PIN)) {
        // don't do anything more:
      return;
    }
    
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    sec =0;
}
 
void loop() 
{
  long time = millis();
  unsigned long currentMillis = millis(); // grab current time
  my_rtc.updateTime();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gy_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gy_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)  

//mySerial.println("[1,2,3,4,5,6,7]");
//  char inChar;
//      done:
//      StaticJsonBuffer<100> jsonBuffer;
//            JsonArray& array = jsonBuffer.createArray();
//            array.add(accelerometer_x);
//            array.add(accelerometer_y);
//            array.add(accelerometer_z);
//            array.add(gy_x);
//            array.add(gy_y);
//            array.add(gy_z);
//            array.add(0);
//            array.printTo(mySerial);
//      
      
       if (digitalRead(BTpin)==HIGH){ 
         Serial.println(F("--- start read ---")); 
             if (SD.exists("test_2.txt")) {
              int sdid; 
              int sdacc_x; 
              int sdacc_y; 
              int sdacc_z; 
              int sdgy_x; 
              int sdgy_y; 
              int sdgy_z; 
              int sdHour; 
              int sdminutes;
              int sdseconds;
              int sdYear;
              myFile = SD.open("test_2.txt"); // open "file.txt" to read data
              // Serial.println("--- Reading start ---");
               while((myFile.available() != -1) && (myFile.available()>0)){
              int sdid = myFile.parseInt();
                 sdacc_x = myFile.parseInt();
                 sdacc_y = myFile.parseInt();
                 sdacc_z = myFile.parseInt();
                 sdgy_x = myFile.parseInt();
                 sdgy_y = myFile.parseInt();
                 sdgy_z = myFile.parseInt();
                 sdHour = myFile.parseInt();
                 sdminutes = myFile.parseInt();
                 sdseconds = myFile.parseInt();
                 sdYear = myFile.parseInt();
//               StaticJsonBuffer<100> jsonBuffer;
//               JsonArray& array = jsonBuffer.createArray();
//                array.add(sdacc_x);
//                array.add(sdacc_y);
//                array.add(sdacc_z);
//                array.add(sdgy_x);
//                array.add(sdgy_y);
//                array.add(sdgy_z);
//                array.add(sdYear);
//                array.printTo(mySerial);
                
                 String data_to_ras = "[" + String(sdacc_x) + "," + String(sdacc_y) + "," + String(sdacc_z) + "," + String(sdgy_x) + "," + String(sdgy_y) + "," + String(sdgy_z) + "," + String(sdYear) + "]\n";
                 mySerial.print(data_to_ras);
                data_to_ras = "\0";
                               
                delay(1000);
               }// close the file:
                myFile.close();
                SD.remove("test_2.txt");
                //goto done;
          }
          else {
    String data_to_ras = "[" + String(accelerometer_x) + "," + String(accelerometer_y) + "," + String(accelerometer_z) + "," + String(gy_x) + "," + String(gy_y) + "," + String(gy_z) + "," + "run" + "]\n";
    mySerial.print(data_to_ras);
    data_to_ras = "\0";
  }
       // done:
       // mySerial.println(accelerometer_x);
       }
        else{Serial.println(F("--- start write ---")); SDCARD_W(accelerometer_x, accelerometer_y, accelerometer_z, gy_x, gy_y, gy_z, my_rtc.hours, my_rtc.minutes,my_rtc.seconds, my_rtc.year);}


//           //check logical condition if avv_blole sent data else save to sdcard
//           if (mySerial.available()){
//            //numChars = ""; 
//            //while(mySerial.available())
//            //numChars =+ inChar;
//           // recvHealthCon();
//          
//            //recvHealthCon(inChar);
//            Serial.println(F("--- start read ---")); 
//            // SDCARD_R (sdacc_x, sdacc_y, sdacc_z, sdgy_x, sdgy_y, sdgy_z, sdHour, sdminutes, sdseconds, sdYear);                
//            }else{ Serial.println(F("--- start write ---"));}// SDCARD_W(accelerometer_x, accelerometer_y, accelerometer_z, gy_x, gy_y, gy_z, my_rtc.hours, my_rtc.minutes,my_rtc.seconds, my_rtc.year);}  
//   
           
             
  gyy[i] = gy_y;
  acy[i] = ac_y;
  i++;
 if(i == 8 ) {
  Maxgy = gyy[0];
  Mingy = gyy[0];
  for(int j = 0 ; j < 8; j++){
    if(Maxgy < gyy[j]) Maxgy = gyy[j]; 
    if(Mingy > gyy[j]) Mingy = gyy[j]; 
    if(acy[j] > sleep_acy || acy[j] < -sleep_acy) sleep =sleep+ 1;
  }
  Gapgy = Maxgy - Mingy;
  if(sleep == 8) { Gapgy = -1;}//sleeping
  if(Gapgy >= 45000 ) {setColor(255, 0, 0);}//running
  if(Gapgy <45000 && Gapgy >12500) {setColor(0, 255, 0);}//walking
  if(Gapgy>0 && Gapgy <=12500) {setColor(0, 0, 255);}//standing
  i = 0;
  sleep = 0;
  } 

  Serial.flush();
  delay(3000); 
}

// led service function
void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(RedPin, red);
  analogWrite(BluePin, green);
  analogWrite(GreenPin, blue);  
}

void SDCARD_W (int acc_x, int acc_y, int acc_z, int gy_x,int gy_y,int gy_z, int Hour, int minutes, int seconds, int Year)
{
         myFile = SD.open("test_2.txt", FILE_WRITE);
        if(myFile)
        {
          myFile.print(sec);
          myFile.print(",");
          myFile.print(acc_x);
          myFile.print(",");
          myFile.print(acc_y);
          myFile.print(",");
          myFile.print(acc_z);
          myFile.print(",");
          myFile.print(gy_x);
          myFile.print(",");
          myFile.print(gy_y);
          myFile.print(",");
          myFile.print(gy_z);
          myFile.print(",");
          myFile.print(Hour);
          myFile.print(",");
          myFile.print(minutes);
          myFile.print(",");
          myFile.print(seconds);
          myFile.print(",");
          myFile.println(Year);
          myFile.close();
        }
        else{
          Serial.println("couldnt open log file");
        }  
  } 

 



