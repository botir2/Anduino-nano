#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(5,6);
#include <PololuLedStrip.h>
PololuLedStrip<9> ledStrip;
#define LED_COUNT 4
rgb_color colors[LED_COUNT];
#define mpu_add 0x68//mpu6050 address
#define sleep_acy 10000
long ac_x, ac_y,ac_z,gy_x,gy_y,gy_z ; //acc, gyro data 
long i=0, Maxgy,Mingy,Gapgy,sleep;
int sec;
long gyy[8], acy[8];
void setup() {
  Wire.begin() ;  //set I2C
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
Serial.begin(9600);
Serial.println("iniializing");
if(!SD.begin(4)){
  Serial.println("Failure");
  return;
}
Serial.println("card ready");
 bluetooth.begin(9600);
sec =0;
}

void loop() {
   long time = millis() ;
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read();
  ac_z = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read();
  gy_y = Wire.read() << 8 | Wire.read() ;
  gy_z = Wire.read() << 8 | Wire.read();
sec = time/1000;


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



