#include <Arduino.h>

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

// #define TX2 PA2
// #define RX2 PA3
// HardwareSerial Serial2 = HardwareSerial(RX2, TX2);

void setup() {
  Serial.begin(9600); //for printing
  Serial3.begin(9600);
  Serial3.flush();
}

void loop() {
  int check_available = Serial3.available();
  while (!check_available) {
    check_available = Serial3.available();
  }

  Serial3.write("Hi moon"); 

  Serial.print("Master received: ");
  Serial.println(Serial3.readString());

  delay(1000);
}