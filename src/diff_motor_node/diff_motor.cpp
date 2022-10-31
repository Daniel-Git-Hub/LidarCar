
#include <pigpiod_if2.h>

#include "diff_motor.h"
#include <stdio.h>

diff_motor::diff_motor(double h, int motLP, int motLM, int motLEn, int motRP, int motRM, int motREn){
    halfBaseWidth = h;
    leftCon.set_pin(motLP, motLM, motLEn);
    rightCon.set_pin(motRP, motRM, motREn);
}

int diff_motor::init(int pigpiod){
    leftCon.init(pigpiod);
    rightCon.init(pigpiod);
    return 0;
}



int diff_motor::set_speed(speed_t speed){
    leftCon.set_speed((speed.linear - halfBaseWidth*speed.angular)*LEFT_GAIN);
    rightCon.set_speed((speed.linear + halfBaseWidth*speed.angular)*RIGHT_GAIN);
    return 0;
}

int diff_motor::set_pwm(int l, int r){
    leftCon.set_pwm(l);
    rightCon.set_pwm(r);
    return 0;
}

int diff_motor::close(){
    leftCon.close();
    rightCon.close();
    return 0;
}