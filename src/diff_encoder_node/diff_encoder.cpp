
#include <pigpiod_if2.h>

#include "diff_encoder.h"
#include <stdio.h>

diff_encoder::diff_encoder(double h, int encL1, int encL2, int encR1, int encR2){
    halfBaseWidth = h;
    leftEnc.set_pin(encL1, encL2);
    rightEnc.set_pin(encR1, encR2);
}

int diff_encoder::init(int pigpiod){

    leftEnc.init(pigpiod);
    rightEnc.init(pigpiod);
    return 0;
}

int diff_encoder::update(double sec){
    leftEnc.update(sec);
    rightEnc.update(sec);
    return 0;
}


speed_t * diff_encoder::get_speed(){
    lastSpeed.linear = (rightEnc.speed + leftEnc.speed)/2;
    lastSpeed.angular = (rightEnc.speed - leftEnc.speed)/halfBaseWidth;
    return &lastSpeed;
}
