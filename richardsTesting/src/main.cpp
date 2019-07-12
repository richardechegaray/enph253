#include <Arduino.h>
#include "IRdecision.h" 

#define LEFT_MOTOR_FW PB_9 
#define LEFT_MOTOR_BW PB_8
#define RIGHT_MOTOR_FW PB_6
#define RIGHT_MOTOR_BW PB_7

#define PIN_LEFT PA0
#define PIN_CENTER PA1
#define PIN_RIGHT PA2

float clockFreq = 100000;
float period = 1000;

IRdecision decision = IRdecision(PIN_LEFT, PIN_CENTER, PIN_RIGHT, 10);

float capSpeed(float speed);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);


void setup(){
  Serial.begin(115200); 
  delay(2000);
  Serial.println("Setup done");
}

void loop() { 

  delay(1000);

  int max_pin;
  max_pin = decision.strongest_signal();

  float left_correlation, center_correlation, right_correlation;
  left_correlation = decision.corrleft;
  center_correlation = decision.corrcenter;
  right_correlation = decision.corrright;

  Serial.print("Left: ");
  Serial.println(left_correlation);

  Serial.print("Middle: ");
  Serial.println(center_correlation);

  Serial.print("Right: ");
  Serial.println(right_correlation);


  Serial.print("Max correlation pin: ");
  Serial.println(max_pin);
}

  // sensor.corr();
  // float duration = sensor.duration;
  // float average = sensor.average;
  // float correlation = sensor.correlation;


  // Serial.print("duration: ");
  // Serial.println(duration*1000000);

  // Serial.print("average: ");
  // Serial.println(average);

  // Serial.print("correlation: ");
  // Serial.println(correlation);
//testing
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) {
  fwLeft = capSpeed(fwLeft);
  fwRight = capSpeed(fwRight);
  bwLeft = capSpeed(bwLeft);
  bwRight = capSpeed(bwRight);

  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

float capSpeed(float speed) {
  if (speed>period)
    return period;
  else if (speed<0)
  return period/10;
  else
    return speed;
}
