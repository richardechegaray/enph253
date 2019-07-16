#include <Arduino.h>
#include "Claw.h"

Claw::Claw(){
    pinMode(CLAW_TOUCH_PIN, INPUT_PULLUP);
    servo_y = Servo(); //up&down
    servo_x = Servo(); //left&right
    servo_y.attach(CLAW_SERVO_PIN_0);
    servo_x.attach(CLAW_SERVO_PIN_1);     
}

void Claw::moveUp(int degrees){
    servo_y.write(90+degrees);  
}
void Claw::moveDown(int degrees){
    servo_y.write(90-degrees); 
}
void Claw::moveLeft(int degrees){
    servo_x.write(90-degrees);
}
void Claw::moveRight(int degrees){
    servo_x.write(90+degrees);
}
void Claw::grabStone(){
    //move forward until we read high
    //start moving upward until we read low
    //move forward until we touch the stone (if we can't, move the claw up and down and repeat)
}

/*
1. raise the claw to a sufficient height
2. keep approaching the pillar (detect with ultrasonic)
3. keep checking until the touch sensor on the claw hits the pillar
4. once we touched, start moving the claw upwards
5. if we read low at any point, move the claw forward a little bit
6. ensure that we read high for a sufficient amount of time
7. wait for reading low
8. when we read low, first make sure we read low for a sufficient amount of time
9. if yes, move the claw forward and try to grab the stone -- if we detect high, great; if we cannot detect high, move the claw down or up until we read high
 */
