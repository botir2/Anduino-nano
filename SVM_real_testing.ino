  #include <SoftwareSerial.h>
#include <virtuabotixRTC.h>
#include "I2Cdev.h"
#include "SdFat.h"
#include <PololuLedStrip.h>

PololuLedStrip<9> ledStrip;
rgb_color colors[4];



#define SD_CS_PIN 10
SdFat SD;
File myFile;
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

#define PIN_CLK 7
#define PIN_DAT 6
#define PIN_RST 5
virtuabotixRTC my_rtc(PIN_CLK, PIN_DAT, PIN_RST);

//for Bluetooth device
int BTpin = 8;
SoftwareSerial mySerial(4, 3); // RX, TX


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 realGyro;         // [x, y, z]            accel sensor measurements

//for classification
int i=0, MaxX, MinX, GapX, MaxY, MinY, GapY, MaxZ, MinZ, GapZ, GapSumm;
uint16_t  rest = 0, run = 0, stand = 0, walk = 0;
int gyx[10], gyy[10], gyz[10];
String StoServer = "\0";
int activity;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // set the BTpin for input
   Serial.begin(9600);
   mySerial.begin(9600);
   pinMode(BTpin, INPUT);   
   //seconds, minutes, hours, day of the week, day of the month, month, year
  //my_rtc.setDS1302Time(0,34,9,32,05,04,2019);
   
   // see if the card is present and can be initialized:
   if (!SD.begin(SD_CS_PIN)) {
      //Serial.println("SD failed");
      return; 
    }
    Wire.begin();
   
  while (!Serial);
  // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

  // verify connection
  delay(3000);
  
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXAccelOffset(-1771);
//    mpu.setYAccelOffset(-310);
//    mpu.setZAccelOffset(1394);
//    mpu.setXGyroOffset(50);
//    mpu.setYGyroOffset(34);
//    mpu.setZGyroOffset(42);

  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
       
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
       
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
  // see if the card is present and can be initialized: 
  delay(500);
  freeRam ();
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

uint32_t timer = millis();
uint32_t SDtimer = millis();
uint32_t BLtimer = millis();

void loop() {
    my_rtc.updateTime();
    
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
/********************************************SD card and bluetooth manipulation fuction*******************************************/ 

         
       #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetGyro(&realGyro, fifoBuffer);
            cls();
            
        #endif
    }
       if (digitalRead(BTpin)==HIGH){         
                   if (SD.exists("test_2.csv")) {
                     myFile = SD.open("test_2.csv"); // open "file.csv" to read data
                      // Serial.println("--- Reading start ---");
                     while((myFile.available() != -1) && (myFile.available()>0)){
                         String sdactivity = myFile.readStringUntil(',');
                         String sdYears = myFile.readStringUntil(',');
                         String sdMonth = myFile.readStringUntil(',');
                         String sdDMonth = myFile.readStringUntil(',');
                         String sdHours = myFile.readStringUntil(',');
                         String sdMins = myFile.readStringUntil(',');
                         String sdSec = myFile.readStringUntil(',');
                         String behaf = myFile.readStringUntil('\n');
                         
                         mySerial.print("[" + sdactivity + "," +  sdYears + "," +  sdMonth + "," +  sdDMonth + "," +  sdHours + "," +  sdMins + "," +  sdSec + "]\n");
                         delay(500); //actually 500 min/s 
                     } // close the file:
                     myFile.close();
                     SD.remove("test_2.csv");
                     delay(100);
                   } else {
                        if (BLtimer > millis())  BLtimer = millis();
                          if (millis() - BLtimer > 5000) {
                            BLtimer = millis(); // reset the timer
                            mySerial.print("["+ String(realGyro.x) + "," + String(realGyro.y) + "," + String(realGyro.z) + "," + String(aa.x) + "," + String(aa.y) + "," + String(aa.z) + "]\n");
                          }
                   }
       }
       else{
        SDCARD_W(activity);  
       }  
       
      if (mySerial.available()){char inChar = (char)mySerial.read();
          Serial.println(inChar);
          if(inChar == '0'){HealthSetColor(0,25,0);}
          else if(inChar == '1'){HealthSetColor(25,25,0);}
          else if(inChar == '2'){HealthSetColor(25,0,0);}
      }
   delay(100);
            
}

void SDCARD_W (int activity)
{
  if (SDtimer > millis())  SDtimer = millis();
     if (millis() - SDtimer > 5000) {
        SDtimer = millis(); // reset the timer
        myFile = SD.open("test_2.csv", FILE_WRITE);
        if(myFile)
        {
          myFile.print(activity);
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
          myFile.print(",");
          myFile.println(0);
          myFile.close();
        }
     }
  } 


/********************************************activity led fuction******************************************************/
void ActivitySetColor(int red1, int green1, int blue1)
{
  rgb_color color;
  color.red = red1;
  color.green = green1;
  color.blue = blue1;

    // Update the colors buffer.
      for(uint16_t i = 0; i < 2; i++)
      {
        colors[i] = color;
      }
      // Write to the LED strip.
      ledStrip.write(colors, 4); 
}

/********************************************health led fuction******************************************************/
void HealthSetColor(int red1, int green1, int blue1)
{
  rgb_color color;
  color.red = red1;
  color.green = green1;
  color.blue = blue1;

    // Update the colors buffer.
      for(uint16_t i = 2; i < 4; i++)
      {
        colors[i] = color;
      }
      // Write to the LED strip.
      ledStrip.write(colors, 4); 
}

/********************************************Classification fuction******************************************************/
void cls()
{
    if (timer > millis())  timer = millis();
     if (millis() - timer > 10) {
        timer = millis(); // reset the timer
            gyx[i] = realGyro.x;
            gyy[i] = realGyro.y;
            gyz[i] = realGyro.z;
            
            i++;
             if(i == 10 ) {
               MaxX = gyx[0];
               MinX = gyx[0];
               MaxY = gyy[0];
               MinY = gyy[0];
               MaxZ = gyz[0];
               MinZ = gyz[0];
               
               for(int j = 0 ; j < 10; j++){
                  if(MaxX < gyx[j]) MaxX = gyx[j]; 
                  if(MinX > gyx[j]) MinX = gyx[j]; 
                  if(MaxY < gyy[j]) MaxY = gyy[j]; 
                  if(MinY > gyy[j]) MinY = gyy[j];
                  if(MaxZ < gyz[j]) MaxZ = gyz[j]; 
                  if(MinZ > gyz[j]) MinZ = gyz[j];
                  
                  //if(roll[j] > 45 || roll[j] < -45) rest =rest + 1;
               }
             
               GapX = MaxX - MinZ;
               GapY = MaxY - MinZ;
               GapZ = MaxZ - MinZ;
               GapSumm = GapX + GapY + GapZ;
               
               if(GapSumm >= 0 && GapSumm <= 100) { stand++; if(stand >= 1){activity = 2; walk = 0; run = 0; ActivitySetColor(10, 10, 10); }} //standing
               if(GapSumm > 100 && GapSumm <= 240) { walk++; if(walk >= 1){activity = 1; stand = 0; walk = 0; run = 0; rest = 0; ActivitySetColor(0, 25, 0); }} //walking
               if(GapSumm > 240)  {run++; if(run >= 1){activity = 0; stand = 0; walk = 0; run = 0; rest = 0; ActivitySetColor(0, 0, 25);  }} //running
               if(stand >= 10) { activity = 3; ActivitySetColor(0, 0, 0); } //resting
               i = 0;
//             rest = 0;
             }
             delay(98);
     }
}
