//a bunch of include's for all the things in lib folder
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
        Robot(){} //constructor - I think pins could be initialized as they're declared so no need to pass everything to the constructor
        typedef enum{
            up_ramp,
            plushie_collection,
            plushie_deposit,
            stone_collection,
            stone_deposit,
            park_stop
        } volatile State;
        State robot_state;
        robot_state = up_ramp;
        //state functions:
        void speed_optimized_pid();
        void regular_pid();
        void follow_IR(); //find the IR sensor (or sensors) with the strongest IR intensity + follow (turn, go straight until the intensity reaches a given value?? need to figure out and code beforehand)
        void stone_collect();
        void stone_deposit();
        void park_stop();
        //interrupt functions (note that some of the interrupts will change the state variable (mosty periodic timer interrupts, external interrupts won't))
        //leave state0:
        static void ramp_end(){
            if(state == up_ramp){
                //TO-DO: Generate this interrupt when the rotary encoders on the wheels turned by a given amount so we know the ramp is over
                state = plushie_collection;
            } else{
                //ignore? because we won't receive this? Miti said to check if we are actually in the state we want to transition from before doing anything
            }
        }
        //leave state1:
        static void deposit_time(){
            if(state == plushie_collection){
                //TO-DO: generate when the time we have spent collecting plushies reaches a certain value
                state = plushie_deposit;
            } else{

            }
        }
        //during state1:
        static void post_nearby(){
            //TO-DO: I believe in this state we don't care whether it's a pillar or a robot that we might run into, we just close doors nmw        
        }
        static void robot_nearby(){
            //see above -- actually, thinking about it again, I really don't think we need this. what are we going to do if a robot is nearby? we can only take logical action when we know we are hit by an obj and from which direction, we can turn accordingly. nearby doesn't help?
        }
        static void post_collision(){
            //TO-DO: closing doors won't help, turn 90 or 180 degrees and continue plushie collection
            //if we run into smth, no longer try to follow the tape because it'll be time consuming to try to find the tape when we can just continue in our plushie coll state until our timer expires to let us deposit the plushies
        }
        static void robot_collision(){
            //same as above
        }
        //state2:
        //we don't care about an object being "nearby"? because even if it is, we'll just continue following IR?
        static void post_collision(){
            //TO-DO: back off a little and and continue with IR following (we don't want to turn 90 or 180?)
        }
        static void robot_collision(){
            //same as above
        }
        //leave state2:
        static void deposit_reached(){
            //TO-DO: when the IR intensity reaches a given level (need to be measured beforehand), drop the plushies and leave this state
        }
        //state3: stone collection
        static void post_collision(){
            //lifting the claw arm up 
        }
        static void robot_collision(){
            //same as post_coll in state1
        }

        
};

