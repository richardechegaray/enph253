#include <Arduino.h>
#include "pid.h"

pid::pid(){
    kp = 1;
    ki = 0; kd = 0; p = 0; i = 0; d = 0; i_limit = 0; prev_error = 0;
}

float pid::output_pid(float error){
    prev_error = error;
    p = kp*error;
    d = kd*(error - prev_error);
    i = ki*(error + i);
    if (i > i_limit) {i = i_limit;};
    if (i < -i_limit) {i = -i_limit;};
    steer = p + i + d;
    return steer;
}