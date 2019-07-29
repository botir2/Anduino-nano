#include <SoftwareSerial.h>
#include <virtuabotixRTC.h>
#include "I2Cdev.h"
#include "SdFat.h"
#include <PololuLedStrip.h>

PololuLedStrip<9> ledStrip;
#define LED_COUNT 4
rgb_color colors[LED_COUNT];



#define SD_CS_PIN SS
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
long i=0, MaxRoll, MinRoll, Gapgy, rest;
long roll[10], acy[10];
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
   //my_rtc.setDS1302Time(0,50,14,9,13,11,2018);
   
   // see if the card is present and can be initialized:
   if (!SD.begin(SD_CS_PIN)) {
      Serial.println("SD failed");
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
    mpu.setXAccelOffset(-1555);
    mpu.setYAccelOffset(-1675);
    mpu.setZAccelOffset(1249);
    mpu.setXGyroOffset(47);
    mpu.setYGyroOffset(-3);
    mpu.setZGyroOffset(35);

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
  //freeRam ();
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

uint32_t timer = millis();

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

         
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
           // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);

            roll[i] = ypr[2] * 180/M_PI;
            //Serial.print("Gyro Y: "); Serial.println(gy_y);  
            //Serial.print("Accl Y: "); Serial.println(ac_y);
            i++;
             if(i == 10 ) {
               MaxRoll = roll[0];
               MinRoll = roll[0];
               for(int j = 0 ; j < 10; j++){
                  if(MaxRoll < roll[j]) MaxRoll = roll[j]; 
                  if(MinRoll > roll[j]) MinRoll = roll[j]; 
                  if(roll[j] > 50 || roll[j] < -50) rest =rest + 1;
               }
             
               Gapgy = MaxRoll - MinRoll;
               if(rest == 10) { activity = 3; setColor(1, 1, 1);  Gapgy = -1;}
               if(Gapgy >= 40)  { activity = 2;setColor(25, 0, 0);}
               if(Gapgy <40 && Gapgy >10) { activity = 1; setColor(0, 25, 0); }
               if(Gapgy>=0 && Gapgy <=10) { activity = 0; setColor(0, 0, 25);}
               i = 0;
               rest = 0;
             }
             delay(98);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetGyro(&realGyro, fifoBuffer);
//            Serial.print("areal\t");
//            Serial.print(realGyro.x);
//            Serial.print("\t");
//            Serial.print(realGyro.y);
//            Serial.print("\t");
//            Serial.println(realGyro.z);
//            delay(100);
//            Serial.print("areal\t");
//            Serial.print(aa.x);
//            Serial.print("\t");
//            Serial.print(aa.y);
//            Serial.print("\t");
//            Serial.println(aa.z);
        #endif 
  
    }
 
       if (digitalRead(BTpin)==HIGH){         
                   if (SD.exists("test_2.txt")) {
                   
                   myFile = SD.open("test_2.txt"); // open "file.txt" to read data
                      // Serial.println("--- Reading start ---");
                     while((myFile.available() != -1) && (myFile.available()>0)){
                         String sdgy_x = myFile.readStringUntil(',');
                         String sdgy_y = myFile.readStringUntil(',');
                         String sdgy_z = myFile.readStringUntil(',');
                         String sdacc_x = myFile.readStringUntil(',');
                         String sdacc_y = myFile.readStringUntil(',');
                         String sdacc_z = myFile.readStringUntil(',');
                         String sdactivity = myFile.readStringUntil(',');
                         String sdYears = myFile.readStringUntil(',');
                         String sdMonth = myFile.readStringUntil(',');
                         String sdDMonth = myFile.readStringUntil(',');
                         String sdHours = myFile.readStringUntil(',');
                         String sdMins = myFile.readStringUntil(',');
                         String sdSec = myFile.readStringUntil(',');
                         String behaf = myFile.readStringUntil('\n');
                         
                         mySerial.print("[" + sdgy_x + "," + sdgy_y + "," + sdgy_z + "," + sdacc_x + "," + sdacc_y + "," + sdacc_z + ","  + sdactivity + "," +  sdYears + "," +  sdMonth + "," +  sdDMonth + "," +  sdHours + "," +  sdMins + "," +  sdSec + "]\n");
                         delay(500);    
                     } // close the file:
                     myFile.close();
                     SD.remove("test_2.txt");
                     delay(100);
                   } else {
                        if (timer > millis())  timer = millis();
                          if (millis() - timer > 1000) {
                            timer = millis(); // reset the timer
                            mySerial.print("[" + String(aa.x) + "," + String(aa.y) + "," + String(aa.z) + "," + String(realGyro.x) + "," + String(realGyro.y) + "," + String(realGyro.x) + "," + String(activity) + "]\n");
                          }
                   }
                      
       // done:
       // mySerial.println(accelerometer_x);
      //delay(100);
       }
       else{
        //delay(10);
        SDCARD_W(realGyro.x, realGyro.y, realGyro.z, aa.x, aa.y, aa.z, activity);
  
       }
    
}

void SDCARD_W (int gy_x, int gy_y, int gy_z, int acc_x, int acc_y, int acc_z, int activity)
{
  if (timer > millis())  timer = millis();
     if (millis() - timer > 1000) {
        timer = millis(); // reset the timer
        myFile = SD.open("test_2.txt", FILE_WRITE);
        if(myFile)
        {
          myFile.print(gy_x);
          myFile.print(",");
          myFile.print(gy_y);
          myFile.print(",");
          myFile.print(gy_z);
          myFile.print(",");
          myFile.print(acc_x);
          myFile.print(",");
          myFile.print(acc_y);
          myFile.print(",");
          myFile.print(acc_z);
          myFile.print(",");
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

  


