#include <Arduino.h>
#include "WheelRotaries.h"

WheelRotaries wr = WheelRotaries();
void setup(){
    Serial.begin(9600);
}

void func(){
  int sum = 0;
  for(int i=0; i<100; i++){
    sum+=i;
  }
}

void loop(){
    Serial.print("Rot 1: ");
    Serial.println(wr.rotaryRotation(wid_1));
    Serial.println();
    Serial.print("Rot 2: ");
    Serial.println(wr.rotaryRotation(wid_2));
    Serial.println();
    func();
    delay(200);
}