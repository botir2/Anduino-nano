#include <SoftwareSerial.h>

SoftwareSerial hm10 (7, 8); // RX, TX  
// Connect HM10      Arduino Uno
//     Pin 1/TXD          Pin 7
//     Pin 2/RXD          Pin 8
unsigned long previousMillis = 0;        // will store last time
const long interval = 500;           // interval at which to delay
static uint32_t tmp = 1; // increment this

void  setup () {
  // put your setup code here, to run once:
  Serial. begin ( 9600 );
  hm10.begin ( 9600 );
  unsigned long previousMillis = 0; 
  //hm10.print("AT+NAMEchumchuq.com");
  delay(3000);
}
 
void  loop () {
  // put your main code here, to run repeatedly:
  while (hm10.available ()) {
    byte  data= hm10.read ();
    Serial.write (data);
  }
  while (Serial.available ()) {

    byte  data= Serial.read ();
    hm10.write (data);
  }

  
    hm10.print("1"); // print this to bluetooth module
  

  delay(500);
}

