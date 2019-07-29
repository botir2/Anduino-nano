#include <SoftwareSerial.h>
#include <virtuabotixRTC.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include "SdFat.h"
SdFat SD;
#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <PololuLedStrip.h>
PololuLedStrip<9> ledStrip;
#define LED_COUNT 4
rgb_color colors[LED_COUNT];

#define mpu_add 0x68
#define sleep_acy 10000
#define PIN_CLK 7
#define PIN_DAT 6
#define PIN_RST 5

/************************************************************ Parameters ****************************************************************/   
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

/************************************************************ VOID setup****************************************************************/   

void setup() 
{
   // set the BTpin for input
    pinMode(BTpin, INPUT);   
 
  // HC-06 default serial speed for communcation mode is 9600
    mySerial.begin(9600);
    Serial.begin(9600);
    // seconds, minutes, hours, day of the week, day of the month, month, year
    //my_rtc.setDS1302Time(0,50,14,9,13,11,2018);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
      // see if the card is present and can be initialized:
     pinMode(chipSelect, OUTPUT);
    if (!SD.begin(SD_CS_PIN)) {
       Serial.print("Sd init...");
      return;
    }
    
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    sec =0;
}

uint32_t timers = millis();

/************************************************************ Parameters in loop ****************************************************************/    
void loop() 
{
   
  
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

/********************************************SD card and bluetooth manipulation fuction*******************************************/   
      
       if (digitalRead(BTpin)==HIGH){ 
               Serial.println(F("--- start read ---"));             
                   if (SD.exists("test_2.txt")) {
                   
                   myFile = SD.open("test_2.txt"); // open "file.txt" to read data
                    // Serial.println("--- Reading start ---");
                   while((myFile.available() != -1) && (myFile.available()>0)){
                   String sdid = myFile.readStringUntil(',');
                   String sdacc_x = myFile.readStringUntil(',');
                   String sdacc_y = myFile.readStringUntil(',');
                   String sdacc_z = myFile.readStringUntil(',');
                   String sdgy_x = myFile.readStringUntil(',');
                   String sdgy_y = myFile.readStringUntil(',');
                   String sdgy_z = myFile.readStringUntil(',');
                   String sdYears = myFile.readStringUntil(',');
                   String sdMonth = myFile.readStringUntil(',');
                   String sdDMonth = myFile.readStringUntil(',');
                   String sdHours = myFile.readStringUntil(',');
                   String sdMins = myFile.readStringUntil(',');
                   String sdSec = myFile.readStringUntil('\n');
                   //String behaf = myFile.readStringUntil('\n');
                  
                       
                       String data_to_ras = "[" + sdacc_x + "," + sdacc_y + "," + sdacc_z + "," +sdgy_x + "," + sdgy_y + "," + sdgy_z + "," +  sdYears + "," +  sdMonth + "," +  sdDMonth + "," +  sdHours + "," +  sdMins + "," +  sdSec + "]\n"; 
                       mySerial.print(data_to_ras);
                       data_to_ras = "\0";
                       delay(500);
                        
                      } // close the file:
                         myFile.close();
                         SD.remove("test_2.txt");
                         delay(100);
                      }
                 else {
                        if (timers > millis())  timers = millis();
                          if (millis() - timers > 3000) {
                            timers = millis(); // reset the timer
                            String data_to_ras = "[" + String(accelerometer_x) + "," + String(accelerometer_y) + "," + String(accelerometer_z) + "," + String(gy_x) + "," + String(gy_y) + "," + String(gy_z) + "]\n";
                            mySerial.print(data_to_ras);
                            data_to_ras = "\0";
                          }
                      }
                      
             // done:
             // mySerial.println(accelerometer_x);
             }
             else{
              //Serial.println(F("--- start write ---"));
              //delay(10);
              SDCARD_W(accelerometer_x, accelerometer_y, accelerometer_z, gy_x, gy_y, gy_z);
             }

//Serial.println(accelerometer_x);



       


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
           
/********************************************simple behavoir clasification fuction*******************************************/             
  gyy[i] = gy_y;
  acy[i] = accelerometer_y;
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
  if(sleep == 8) {Serial.println("sleep"); setColor(1, 1, 1);Gapgy = -1;}//sleeping
  if(Gapgy >= 45000 ) {Serial.println("run");setColor(25, 0, 0);}//running
  if(Gapgy <45000 && Gapgy >12500) {Serial.println("Walk");setColor(0, 25, 0);}//walking
  if(Gapgy>0 && Gapgy <=12500) {Serial.println("Stand");setColor(0, 0, 25);}//standing
  i = 0;
  sleep = 0;
  } 

  Serial.flush();
  delay(100); 
}

/********************************************led condirion fuction******************************************************/
void setColor(int red1, int green1, int blue1)
{
 rgb_color color;
  color.red = red1;
  color.green = green1;
  color.blue = blue1;

    // Update the colors buffer.
      for(uint16_t i = 0; i < LED_COUNT; i++)
      {
        colors[i] = color;
      }
      // Write to the LED strip.
      ledStrip.write(colors, LED_COUNT); 
}



/********************************************write sd card fuction******************************************************/

void SDCARD_W (int acc_x, int acc_y, int acc_z, int gy_x,int gy_y,int gy_z)
{
  if (timers > millis())  timers = millis();
     if (millis() - timers > 3000) {
        timers = millis(); // reset the timer
        //sprintf(timer,"%02d-%02d-%d %02d:%02d:%02d",my_rtc.year, my_rtc.month, my_rtc.dayofmonth, my_rtc.hours, my_rtc.minutes, my_rtc.seconds);
        myFile = SD.open("test_2.txt", FILE_WRITE);
        if(myFile)
        {
//          String data_RW_to_sd = String(sec) + "," + String(acc_x) + "," + String(acc_y) + "," + String(acc_z) + "," + String(gy_x) + "," + String(gy_y) + ","  + String(gy_z) + "," + 
//          String(my_rtc.year)+ "," + String(my_rtc.month)+ "," + String(my_rtc.dayofmonth)+ "," + String(my_rtc.hours)+ "," + String(my_rtc.minutes)+ "," + 
//          String(my_rtc.seconds)+ "\n";
//          myFile.println(data_RW_to_sd);
//          myFile.close();

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
          myFile.print(my_rtc.year);
          myFile.print(",");
          myFile.print(my_rtc.month);
          myFile.print(",");
          myFile.print(my_rtc.dayofmonth);
          myFile.print(",");
          myFile.print(my_rtc.hours);
          myFile.print(",");
          myFile.print(my_rtc.minutes);
          myFile.print(",");
          myFile.print(my_rtc.seconds);
          myFile.println("\n");
          myFile.close();
        }
        
     }
  } 

//void loops(String s)
//{
//   if (timers > millis())  timers = millis();
//         if (millis() - timers > 500) {
//          timers = millis(); // reset the timer
//          //if (tinyGPS.location.isValid()) {
//            Serial.flush();
//            StoServer = "\0";
//        }
//}






