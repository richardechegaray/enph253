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
  //Serial2.begin(9600);
  Serial3.begin(9600);
  Serial.println("Setup done, MASTER");
}

void loop() {
  //Serial.println("in loop");
  //delay(1000);
  //put your main code here, to run repeatedly:
  //Serial.println("Ready to speak!");
  int check_available = Serial3.available();
  while (!check_available) {
    //Serial.println("No messages available");
    check_available = Serial3.available();
  }
  Serial.print("Master received: ");
  Serial.println(Serial3.readString());
  Serial3.print("Master sent");
  delay(500);
  // String message = Serial3.readString();
  // Serial.print("My friend (SLAVE) said: ");
  // Serial.println(message);
  // Serial3.println("Hi moon"); //master says
  // delay(5000);
}