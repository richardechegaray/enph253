#include <Arduino.h>
#include "IRsensor.h"
#define SIZE 200

float raw_sample [SIZE];
float waveform [2*SIZE];

IRsensor::IRsensor(int set_pin, int mode){
    pin = set_pin;
    pinMode(pin, INPUT);

    sample_size = SIZE; // the amount of values to collect into array for reading

    mode = mode;
    if (mode == 1){
        power = 3;
    }
    if (mode == 10){
        power = 4;
    }
    else {
        Serial.println("Error: incorrect mode");
    }
}

void IRsensor::sample(int pin){
    for (int i = 0; i < sample_size; i++){
        raw_sample[i] = analogRead(pin);
    }
}

void IRsensor::corr(){
    time_start = micros();
    sample(pin);
    time_end = micros();
    duration = time_end - time_start;
    time_per_sample = duration/sample_size;

    // find average of samples 
    float sum_sample = 0.0;
    for (int i = 0; i < sample_size; i++){
        sum_sample += raw_sample[i];
    }
    average = sum_sample/sample_size;

    // subtract every value in the array by that to centre it
    for (int i = 0; i < sample_size; i++){
        raw_sample[i] = raw_sample[i] - average;
    }

    // generate mode array
    for (int i = 0; i < (2*sample_size); i++){
        waveform[i] = sin(2*PI*pow(10,power)/time_per_sample);
    }

    // compare sample array to the generated array
    // correlation = IRsamples*waveform
    float corr_values[2*sample_size] = {};
    for (int m = 0; m < (sample_size); m++){
        for (int n = 0; n < (2*sample_size); n++){
            corr_values[m] = raw_sample[n]*waveform[m+n];
        }
    }

    for (int i = 0; i < (2*sample_size); i++){
        correlation += corr_values[i];
    }
}