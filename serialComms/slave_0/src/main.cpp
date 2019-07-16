#include <Arduino.h>

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

// #define TX2 PA2
// #define RX2 PA3
// HardwareSerial Serial2 = HardwareSerial(RX2, TX2);

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial3.flush();
}

void loop() {
  int check_available = Serial3.availableForWrite();
  while (!check_available)
    check_available = Serial3.availableForWrite();  

  Serial3.write("Hello world"); //writing

  Serial.print("Slave received: ");   //printing out recieved stuff
  Serial.println(Serial3.readString());

  delay(1000);
}