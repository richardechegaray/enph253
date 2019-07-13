#include <Arduino.h>

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

// #define TX2 PA2
// #define RX2 PA3
// HardwareSerial Serial2 = HardwareSerial(RX2, TX2);

void setup() {
  // put your setup code here, to run once:
  //initUSART();
  Serial.begin(9600);
  Serial3.begin(9600);
  //Serial2.begin(9600);
  Serial.println("Setup done, SLAVE");
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(5000);
  int check_available = Serial3.availableForWrite();
  while (!check_available){
    check_available = Serial3.availableForWrite();  
  }
  Serial3.print("Slave sent");
  Serial.print("Slave received: ");
  Serial.println(Serial3.readString());
  delay(500);
  //Serial.println("Sending to other BP...");
  //Serial3.write("Hello world"); //slave says
  //Serial2.print("Hello world");
  //delay(3000);
  //Serial.print("My friend said: " + Serial2.readString());
  //Serial.print("My friend (MASTER) said: " + Serial3.readString());
}