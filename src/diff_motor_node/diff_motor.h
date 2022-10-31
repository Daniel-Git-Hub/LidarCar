#ifndef DIFF_MOTOR_H
#define DIFF_MOTOR_H

#include "motor_controller.h"

#include "board_def.h"

class diff_motor {

    private:
    motor_controller leftCon;
    motor_controller rightCon;
    double halfBaseWidth;

    public:

    diff_motor();
    diff_motor(double h, int motLP, int motLM, int motLEn, int motRP, int motRM, int motREn);
    int init(int pigpiod);
    
    //speed_t is small so its a paramater/returned instead of a pointer
    int set_speed(speed_t speed);
    int set_pwm(int l, int r);
    int close();

};


#endif