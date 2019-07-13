#include <Arduino.h>
//#include "HCSR04.h"
#define TRIG PB3 //NOTE!!!! US sensor pins HAVE TO BE 5V TOLERANT!!!!! PA1, PA0 DO NOT WORK!!!!
#define ECHO PB4
#define threshold 10
#define CM_MCS 0.0343
long duration;
long distance;
//note2: there should only be 1 common ground!!! don't connect all 3!!!

//UltraSonicDistanceSensor Usensor(TRIG, ECHO);
//4 pins: 5v, trigger, echo, gnd
//send a pulse from bp to trigger
//receive from echo
//this week experiment with the range and circuit connections of the sensors; later on I will refactor the code into proper classes and add our own functions for our purposes (US sensor will be spinning on a servo motor)

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  duration = 0;
  distance = 0;
}

void loop() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) * CM_MCS;
  int check_available = Serial3.availableForWrite();
  //Serial.println("available 1");
  while (!check_available){
    Serial.println("serial3 not available");
    check_available = Serial3.availableForWrite();  
  }
  //Serial.println("available 2");
  //Serial.println(Serial3.readString());
  Serial3.print("distance: ");
  Serial3.println(distance);
  Serial.println("passed info!");
  // Serial.print("cm: ");
  // Serial.println(distance);
  delay(500);
}
//   //*/

//   //Serial.println(Usensor.measureDistanceCm());
// }