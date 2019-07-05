//a bunch of include's for all the things in lib folder
#include "RotaryEncoder.h"
#include "IRSensor.h"
#include "ReflSensor.h"
#include "USensor.h"
#include "TouchSensor.h"

class Robot{
    private:
        RotaryEncoder right_wheel_rotary;
        RotaryEncoder left_wheel_rotary;
        //declare all sensors
    public:
        Robot(){}
        enum{
            up_ramp,
            plushie_collection,
            plushie_deposit,
            stone_collection,
            stone_deposit,
            park_stop
        } volatile state;

        //state functions:
        void speed_optimized_pid();
        void regular_pid();
        void follow_IR(); //find the IR sensor (or sensors) with the strongest IR intensity + follow

        //interrupt functions (note that some of the interrupts will change the state variable (mosty periodic timer interrupts, external interrupts won't))
        //state0:
        static void ramp_end(){
            if(state == up_ramp){
                //check the distance from the rotary encoders on the wheels beforehand
                //how to generate an interrupt when the rotary distance reaches a certain value?
                state = plushie_collection;
            } else{
                //ignore? because we won't receive this? Miti said to check if we are actually in the state we want to transition from before doing anything
            }
        }
        //state1:
        static void deposit_time(){
            if(state == plushie_collection){
                //check the time we have spent collecting plushies beforehand
                state = plushie_deposit;
            } else{

            }
        }
};

