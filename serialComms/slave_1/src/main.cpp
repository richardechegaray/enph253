#include <Arduino.h>

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);
//char buffer[10];
void setup() {
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("Slave: exiting setup");
}

void loop() {
  //Serial.println("Slave: in loop");
  int check_available = Serial3.available();
  //Serial.println("Slave: about to enter while");
  while (!check_available) {
    Serial.println("No messages available");
    check_available = Serial3.available();
  }
  //Serial.print("Received cm: ");
  //size_t msg_received = Serial3.readBytes(buffer, 2);
  Serial.println(Serial3.readString());
  // String dist = Serial3.readString();
  // Serial.println(dist);
  delay(500);
  //buffer.clear
}

// #include "Arduino.h"

// // Set LED_BUILTIN if it is not defined by Arduino framework
// // #define LED_BUILTIN 13

// void setup()
// {
//  // initialize LED digital pin as an output.
//  pinMode(LED_BUILTIN, OUTPUT);
//  Serial.begin(115200);
// }

// void loop()
// {
//  // turn the LED on (HIGH is the voltage level)
//  digitalWrite(LED_BUILTIN, HIGH);

//  delay(1000);

//  // turn the LED off by making the voltage LOW
//  digitalWrite(LED_BUILTIN, LOW);

//   // wait for a second
//  delay(1000);

//  Serial.println("flashed");
// }
