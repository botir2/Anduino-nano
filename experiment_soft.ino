//#include <SoftwareSerial.h>
//#include <ArduinoJson.h>
#include <virtuabotixRTC.h>
#include "I2Cdev.h"
#include "SdFat.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

SdFat SD;
File myFile;

#define PIN_CLK 7
#define PIN_DAT 6
#define PIN_RST 5
virtuabotixRTC my_rtc(PIN_CLK, PIN_DAT, PIN_RST);

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 realGyro;         // [x, y, z]            accel sensor measurements


//for classification
long i=0, Maxgy, Mingy, Gapgy, sleep;
long gyy[10], acy[10];
String StoServer = "\0";
String activity = "\0";

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
    //Wire.begin();
    Serial.begin(9600);

    if (!SD.begin(4)) {
        // don't do anything more:
      return;
    }

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    delay(3000);
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1771);
    mpu.setYAccelOffset(-310);
    mpu.setZAccelOffset(1394);
    mpu.setXGyroOffset(50);
    mpu.setYGyroOffset(34);
    mpu.setZGyroOffset(42);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


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
            
            
//            gyy[i] = ypr[2] * 180/M_PI;
//            //Serial.print("Gyro Y: "); Serial.println(gy_y);  
//            //Serial.print("Accl Y: "); Serial.println(ac_y);
//            i++;
//             if(i == 10 ) {
//               Maxgy = gyy[0];
//               Mingy = gyy[0];
//               for(int j = 0 ; j < 10; j++){
//                  if(Maxgy < gyy[j]) Maxgy = gyy[j]; 
//                  if(Mingy > gyy[j]) Mingy = gyy[j]; 
//                  if(gyy[j] > 50 || gyy[j] < -50) sleep =sleep + 1;
//               }
//             
//               Gapgy = Maxgy - Mingy;
//               if(sleep == 10) {Serial.println("resting"); Gapgy = -1;}
//               if(Gapgy >= 70)  {Serial.println("running");}
//               if(Gapgy <70 && Gapgy >20) {Serial.println("Walking");}
//               if(Gapgy>=0 && Gapgy <=20) {Serial.println("Standing");}
//               i = 0;
//               sleep = 0;
//             }
//             delay(98);
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
        
        StoServer = String(ypr[0] * 180/M_PI) + "," + String(ypr[1] * 180/M_PI) + "," + String(ypr[2] * 180/M_PI) + "," + String(realGyro.x) + "," + String(realGyro.y) + "," + String(realGyro.z) + "," + String(aa.x) + "," + String(aa.y) + "," + String(aa.z) +
        "," + String(my_rtc.year) + "." + String(my_rtc.month) + "." + String(my_rtc.dayofmonth) + " " + String(my_rtc.hours) + ":" + String(my_rtc.minutes) + ":" + String(my_rtc.seconds);
        //SDCARD_W(StoServer);
        loops(StoServer);
        StoServer = "\0";
        delay(100);
    }
}

void loops(String s)
{
   if (timer > millis())  timer = millis();
         if (millis() - timer > 200) {
          timer = millis(); // reset the timer
          //if (tinyGPS.location.isValid()) {
            Serial.println(s); Serial.flush();
            StoServer = "\0";
        }
}


void SDCARD_W (String data) {
  if (timer > millis())  timer = millis();

  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    myFile = SD.open("data.csv", FILE_WRITE);
    if(myFile) {
      myFile.println(data);
      data = "\0";
      myFile.close();
    } else {
            Serial.println("couldnt open log file");
            data = "\0";
    }  
  }
} 
