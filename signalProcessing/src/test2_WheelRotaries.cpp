// //check if the rotary interrupt can count properly when main is doing smth else
// #include <Arduino.h>
// #include "WheelRotaries.h"
// WheelRotaries wr = WheelRotaries();
// unsigned long previousTime = 0;
// unsigned long currentTime;
// void setup(){
//   Serial.begin(9600);
// }

// void loop(){
//   currentTime = millis();
//   while(currentTime - previousTime <= 100) {
//     currentTime = millis();    
//   }
//   Serial.println(wr.wheelDistance(wid_1));
//   Serial.println();
//   delay(1000);
//   previousTime = currentTime;
// }