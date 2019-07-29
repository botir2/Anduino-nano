#include <SoftwareSerial.h>
SoftwareSerial BTserial(3, 4); // RX | TX

 
char c = ' ';
 
// BTconnected will = false when not connected and true when connected
boolean BTconnected = false;
 
// connect the STATE pin to Arduino pin D4
int BTpin = 10;
 
 
void setup() 
{
    // set the BTpin for input
    pinMode(BTpin, INPUT);   
 
    // start serial communication with the serial monitor on the host computer
    Serial.begin(9600);
    Serial.println("Arduino is ready");
    Serial.println("Connect the HC-05 to an Android device to continue");
 
    // wait until the HC-05 has made a connection
   
     
 
   // Serial.println("HC-05 is now connected");
    Serial.println("");
 
    // Start serial communication with the bluetooth module
    // HC-05 default serial speed for communication mode is 9600 but can be different
    BTserial.begin(9600);  
}
 
void loop()
{

   // wait until the HC-05 has made a connection
   
    if (digitalRead(BTpin)==HIGH){ Serial.println(digitalRead(BTpin));}
    else {Serial.println(digitalRead(BTpin));}
    
// BTserial.println("[1,2,3,4,5,6,7]");
 
    // Keep reading from the HC-05 and send to Arduino Serial Monitor
//    if (BTserial.available())
//    {  
//        c = BTserial.read();
//        Serial.write(c);
//    }
// 
//    // Keep reading from Arduino Serial Monitor input field and send to HC-05
//    if (Serial.available())
//    {
//        c =  Serial.read();
//        BTserial.write(c);  
//    }
    delay(1000);
}     
