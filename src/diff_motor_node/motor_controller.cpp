#include <stdlib.h>
#include <stdio.h>

#include <pigpiod_if2.h>
#include "motor_controller.h"
#include <math.h>
#include "board_def.h"


motor_controller::motor_controller(int p, int m, int en){
    set_pin(p, m, en);
}

int motor_controller::set_pin(int p, int m, int en){
    pinP = p;
    pinM = m;
    pinEn = en;
    return 0;
}

int motor_controller::init(int pig){
    pigPio = pig;

    set_mode(pigPio, pinP, PI_OUTPUT);
    set_mode(pigPio, pinM, PI_OUTPUT);
    set_mode(pigPio, pinEn, PI_OUTPUT);
    return 0;
}

int motor_controller::set_speed(double speed){
    lastSpeed = speed;

    int p = (int)round(speed*LINEAR_GAIN);
    // printf("PWM %d, %d, %lf\n", pwm, (int)round(1000000*(abs(speed)*5 - 0.6)), round(1000000*(abs(speed)*5 - 0.6)));
    
    // else if(reverse){
    //     gpio_write(pigPio, pinM, 0);
    //     gpio_write(pigPio, pinP, 1);
    //     hardware_PWM(pigPio, pinEn, PMW_FREQUENCY, pwm);
    // }else if(!reverse){
    //     gpio_write(pigPio, pinP, 0);
    //     gpio_write(pigPio, pinM, 1);
    //     hardware_PWM(pigPio, pinEn, PMW_FREQUENCY , pwm);
    // }

    return set_pwm(p);
}

int motor_controller::set_pwm(int p){
    pwm = p;
    int8_t reverse = pwm > 0;
    p = abs(p);

    if(pwm > 1000000){
        pwm = 1000000;
    } else if(pwm && pwm < PWM_MIN){
        pwm = PWM_MIN;
    }

    if(pwm == 0){ //brake
        gpio_write(pigPio, pinP, 0);
        gpio_write(pigPio, pinM, 0);
        gpio_write(pigPio, pinEn, 1);
    }else{ //move
        gpio_write(pigPio, pinM, !reverse);
        gpio_write(pigPio, pinP, reverse);
        hardware_PWM(pigPio, pinEn, PMW_FREQUENCY, p);
    }
    return p;
}

int motor_controller::close(){
    set_mode(pigPio, pinP, PI_INPUT);
    set_mode(pigPio, pinM, PI_INPUT);
    set_mode(pigPio, pinEn, PI_INPUT);
    return 0;
}