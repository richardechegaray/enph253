// //master
// #include <Arduino.h>
// #include "WheelRotaries.h"


// #define TX3 PB10
// #define RX3 PB11
// HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

// WheelRotaries wr = WheelRotaries();
// unsigned long previousTime = 0;
// unsigned long currentTime;
// long distance = 0;

// void setup(){
//   Serial.begin(9600);
//   Serial3.begin(9600);
// }

// void loop(){
//   currentTime = millis();
//   while(currentTime - previousTime <= 100) {
//     currentTime = millis();    
//   }
//   distance = wr.wheelDistance(wid_1);
//   Serial.println(distance);
//   Serial.println();
//   delay(1000);
//   int check_available = Serial3.available();
//   while (!check_available) {
//     check_available = Serial3.available();
//   }
//   if(distance > 10 || distance < -10){
//       Serial3.print(distance);
//   }
//   previousTime = currentTime;
// }