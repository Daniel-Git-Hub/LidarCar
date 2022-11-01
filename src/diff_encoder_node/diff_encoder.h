#ifndef DIFF_ENCODER_H
#define DIFF_ENCODER_H

#include "rotary_encoder.h"
#include "board_def.h"

class diff_encoder {

    private:


    double baseWidth;
    speed_t lastSpeed;

    public:
    motor_encoder leftEnc;
    motor_encoder rightEnc;
    
    diff_encoder();
    diff_encoder(double h, int encL1, int encL2, int encR1, int encR2);
    int init(int pigpiod);
    
    int update(double sec);

    //speed_t is small so its a paramater/returned instead of a pointer
    speed_t * get_speed();

    int close();

};


#endif