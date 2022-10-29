
#include <pigpiod_if2.h>
#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define MOTOR_ENABLE 16
#define MOTOR_P 21
#define MOTOR_M 20
// #define MOTOR_IN_RANGE 255 //from -255 to 255

#define MAX_SPEED_IN 2
#define MIN_SPEED_IN -1.0
// #define MIN_PWM      128
#define MIN_PWM      180

#define SERVO_PIN 18
#define SERVO_MID 1820
#define SERVO_RANGE 400.0
#define SERVO_MAX_ANGLE (18.0*M_PI/180.0)

#define SERVO_IN_RANGE 255 //from -255 to 255

int pigPio = -1;

int car_init(int p){
    pigPio = p;
    set_mode(pigPio, MOTOR_ENABLE, PI_OUTPUT);
    set_mode(pigPio, MOTOR_P, PI_OUTPUT);
    set_mode(pigPio, MOTOR_M, PI_OUTPUT);
    set_mode(pigPio, SERVO_PIN, PI_OUTPUT);
    gpio_write(pigPio, MOTOR_ENABLE, 0);
    set_servo_pulsewidth(pigPio, SERVO_PIN, SERVO_MID);
    set_PWM_frequency(pigPio, MOTOR_ENABLE, 10);
    return 0;
}


int car_move(double speedDouble){
    if(speedDouble < MIN_SPEED_IN){
        speedDouble = MIN_SPEED_IN;
    }else if(speedDouble > MAX_SPEED_IN){
        speedDouble = MAX_SPEED_IN;
    }
    
    int reverse = speedDouble > 0;
    speedDouble = abs(speedDouble);

    // if(!reverse){
    //     speedDouble = speedDouble*2/3;
    // }

    int speed = (int)round(speedDouble * 255 / MAX_SPEED_IN);
    // printf("Speed %d\n", speed);
    if(speed && speed < MIN_PWM){
        speed = MIN_PWM;
    }

    if(speed == 0){
        gpio_write(pigPio, MOTOR_P, 0);
        gpio_write(pigPio, MOTOR_M, 0);
        gpio_write(pigPio, MOTOR_ENABLE, 1);
    }else if(reverse) {
        gpio_write(pigPio, MOTOR_M, 0);
        gpio_write(pigPio, MOTOR_P, 1);
        set_PWM_dutycycle(pigPio, MOTOR_ENABLE, speed);
    }else {
        gpio_write(pigPio, MOTOR_P, 0);
        gpio_write(pigPio, MOTOR_M, 1);
        set_PWM_dutycycle(pigPio, MOTOR_ENABLE, speed);
    }
    return 0;
}

int car_rotate(double magnitude){
    if(magnitude < -SERVO_MAX_ANGLE){
        magnitude = -SERVO_MAX_ANGLE;
    }else if(magnitude > SERVO_MAX_ANGLE){
        magnitude = SERVO_MAX_ANGLE;
    }

    int pulseWidth = SERVO_MID + (int)round(magnitude*SERVO_RANGE/SERVO_MAX_ANGLE);
    // ROS_WARN("Pulse %lf %d\n", magnitude, pulseWidth);
    // int servo = SERVO_MID + SERVO_RANGE*magnitude/SERVO_IN_RANGE;

    set_servo_pulsewidth(pigPio, SERVO_PIN, pulseWidth);

    return 0;
}

int car_close(){
    //Set pins to open-drain
    set_mode(pigPio, MOTOR_ENABLE, PI_INPUT);
    set_mode(pigPio, MOTOR_P, PI_INPUT);
    set_mode(pigPio, MOTOR_M, PI_INPUT);
    set_mode(pigPio, SERVO_PIN, PI_INPUT);
    return 0;
}
