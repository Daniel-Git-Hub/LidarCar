
#include <pigpiod_if2.h>
#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define MOTOR_ENABLE 12
#define MOTOR_P 21
#define MOTOR_M 20
// #define MOTOR_IN_RANGE 255 //from -255 to 255

#define MAX_SPEED_IN 0.5
#define MIN_SPEED_IN -0.5
// #define MIN_PWM      128
#define MIN_PWM      450000
#define MAX_PWM      650000
#define ANGLE_BOOST_PWM  100000
#define FREQ_PWM     10

#define SERVO_PIN 18
#define SERVO_MID 1820
#define SERVO_RANGE 400.0
#define SERVO_MAX_ANGLE (18.0*M_PI/180.0)


int pigPio = -1;

int speed = 0;
int reverse = 0;
int carState = 1;
ros::Time speedLastTime;
ros::Duration speedWaitTime(0, 1000*1000*1000);

int car_init(int p){
    pigPio = p;
    set_mode(pigPio, MOTOR_ENABLE, PI_OUTPUT);
    set_mode(pigPio, MOTOR_P, PI_OUTPUT);
    set_mode(pigPio, MOTOR_M, PI_OUTPUT);
    set_mode(pigPio, SERVO_PIN, PI_OUTPUT);
    gpio_write(pigPio, MOTOR_ENABLE, 0);
    // set_servo_pulsewidth(pigPio, SERVO_PIN, SERVO_MID);
    // set_PWM_frequency(pigPio, MOTOR_ENABLE, 10);
    return 0;
}

int car_move(double speedDouble, double angle){

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
    if(speedDouble){
        speed = (int)round( ((speedDouble/MAX_SPEED_IN) * (MAX_PWM-MIN_PWM) + MIN_PWM) + (ANGLE_BOOST_PWM * abs(angle)/SERVO_MAX_ANGLE ) );
        
    }else{
        speed = 0;
    }
    
    
    
    // speed = (int)round(speedDouble * MAX_PWM / MAX_SPEED_IN);
    // if(speed && speed < MIN_PWM){
    //     speed = MIN_PWM;
    // }

    // if(carState){
    //     car_active();
    // }

    ROS_WARN("Speed %d - %lf\n", speed, angle);

    if(speed == 0){
        gpio_write(pigPio, MOTOR_P, 0);
        gpio_write(pigPio, MOTOR_M, 0);
        gpio_write(pigPio, MOTOR_ENABLE, 1);
    }else if(reverse) {
        gpio_write(pigPio, MOTOR_M, 0);
        gpio_write(pigPio, MOTOR_P, 1);
        hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
        // set_PWM_dutycycle(pigPio, MOTOR_ENABLE, speed);
    }else {
        gpio_write(pigPio, MOTOR_P, 0);
        gpio_write(pigPio, MOTOR_M, 1);
        hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
        // set_PWM_dutycycle(pigPio, MOTOR_ENABLE, speed);
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
