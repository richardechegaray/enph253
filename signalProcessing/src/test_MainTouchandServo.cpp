// #include <Arduino.h>
// #include "MainTouchSensors.h"
// #include "ClawTouchSensor.h"
// #include "Servo.h"
// #define SERVO_PIN PB12
// MainTouchSensors ts = MainTouchSensors();
// Servo myservo = Servo();

// void setup(){
//     Serial.begin(9600);
//     myservo.attach(SERVO_PIN);
// }

// void loop(){
//   if(ts.getTouched(tid_0) == 1){
//     myservo.write(60);
//     delay(2000);
//     myservo.write(90);
//     Serial.println("touched to 0!");
//   }
//   if(ts.getTouched(tid_1) == 1){
//     Serial.println("touched to 1!");
//     myservo.write(120);
//     delay(2000);
//     myservo.write(90);
//   }   
// }