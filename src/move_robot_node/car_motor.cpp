
#include <pigpiod_if2.h>
#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define MOTOR_ENABLE 19
#define MOTOR_P 21
#define MOTOR_M 20
// #define MOTOR_IN_RANGE 255 //from -255 to 255

#define MAX_SPEED_IN 0.75
#define MIN_SPEED_IN -0.75
// #define MIN_PWM      128
#define MIN_PWM          300000
#define MAX_PWM          900000
#define ANGLE_BOOST_PWM   50000
// #define ANGLE_BOOST_PWM  200000
#define FREQ_PWM     10

#define SERVO_PIN 18
#define SERVO_MID 1820
#define SERVO_RANGE 400.0
#define SERVO_MAX_ANGLE (18.0*M_PI/180.0)
#define SERVO_FREQ 50.0

#define SPEED_DIFF 10
#define RAMP_TIME 0.3

typedef enum {
    CAR_STATE_OFF,
    CAR_STATE_FORWARD_RAMP,
    CAR_STATE_FORWARD,
    CAR_STATE_BACKWARD_RAMP,
    CAR_STATE_BACKWARD,

} car_state_t;

int pigPio = -1;

int speed = 0;
int reverse = 0;
car_state_t carState = CAR_STATE_OFF;

int prevSpeed = 0;
int prevDir = 0;
double rampStart = 0;


int car_init(int p){
    pigPio = p;
    set_mode(pigPio, MOTOR_ENABLE, PI_OUTPUT);
    set_mode(pigPio, MOTOR_P, PI_OUTPUT);
    set_mode(pigPio, MOTOR_M, PI_OUTPUT);
    set_mode(pigPio, SERVO_PIN, PI_OUTPUT);

    gpio_write(pigPio, MOTOR_ENABLE, 0);
    // set_servo_pulsewidth(pigPio, SERVO_PIN, SERVO_MID);
    set_PWM_frequency(pigPio, MOTOR_P, FREQ_PWM);
    set_PWM_frequency(pigPio, MOTOR_M, FREQ_PWM);
    return 0;
}


int car_update(){

    // if(carState == CAR_STATE_BACKWARD_RAMP || carState == CAR_STATE_FORWARD_RAMP ){
    //     if(time_time() - rampStart > RAMP_TIME){
    //         ROS_WARN("CAR SLOW %d", speed);
    //         hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
    //         carState = (CAR_STATE_BACKWARD_RAMP ? CAR_STATE_BACKWARD : CAR_STATE_FORWARD);
    //     }

    // }

    return 0;
}

int car_move(double speedDouble, double angle){

    if(speedDouble < MIN_SPEED_IN){
        speedDouble = MIN_SPEED_IN;
    }else if(speedDouble > MAX_SPEED_IN){
        speedDouble = MAX_SPEED_IN;
    }
    reverse = speedDouble > 0;
    speedDouble = abs(speedDouble);


    if(speedDouble){
        // speed = (int)round( ((speedDouble/MAX_SPEED_IN) * (MAX_PWM-MIN_PWM) + MIN_PWM) );
        speed = (int)round( ((speedDouble/MAX_SPEED_IN) * (MAX_PWM-MIN_PWM) + MIN_PWM) + (ANGLE_BOOST_PWM * abs(angle)/SERVO_MAX_ANGLE ) );
        
    }else{
        speed = 0;
    }
    
    if(prevSpeed == 0 && speed){
        set_mode(pigPio, MOTOR_ENABLE, PI_OUTPUT);
        set_mode(pigPio, MOTOR_P, PI_OUTPUT);
        set_mode(pigPio, MOTOR_M, PI_OUTPUT);
        prevSpeed = speed;
    }

    if(speed){
        // if(reverse){
        //     gpio_write(pigPio, MOTOR_M, 0);
        //     set_PWM_dutycycle(pigPio, MOTOR_P, speed*255/1000000);
        
        // }else{
        //     gpio_write(pigPio, MOTOR_P, 0);
        //     set_PWM_dutycycle(pigPio, MOTOR_M, speed*255/1000000);
        // }
        gpio_write(pigPio, MOTOR_P, reverse);
        gpio_write(pigPio, MOTOR_M, !reverse);
        hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);

        // if(reverse){
        //     if(carState != CAR_STATE_FORWARD && carState != CAR_STATE_FORWARD_RAMP ){
        //         carState = CAR_STATE_FORWARD_RAMP;
        //         rampStart = time_time();
        //         // hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
        //         gpio_write(pigPio, MOTOR_ENABLE, 1);
        //         return 0;
        //     }
        //     if(carState == CAR_STATE_FORWARD_RAMP){
        //         return 0;
        //     }
        // }else{
        //     if(carState != CAR_STATE_BACKWARD && carState != CAR_STATE_BACKWARD_RAMP ){
        //         carState = CAR_STATE_BACKWARD_RAMP;
        //         rampStart = time_time();
        //         // hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
        //         gpio_write(pigPio, MOTOR_ENABLE, 1);
        //         return 0;

        //     }
        //     if(carState == CAR_STATE_BACKWARD_RAMP){
        //         return 0;
        //     }
        // }
        // hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
    }else{

        if(prevSpeed != 0){
            carState = CAR_STATE_OFF;
            set_mode(pigPio, MOTOR_ENABLE, PI_INPUT);
            set_mode(pigPio, MOTOR_P, PI_INPUT);
            set_mode(pigPio, MOTOR_M, PI_INPUT);
            // gpio_write(pigPio, MOTOR_P, 0);
            // gpio_write(pigPio, MOTOR_M, 0);
            // gpio_write(pigPio, MOTOR_ENABLE, 0);
        }

    }

    // ROS_WARN("Speed %lf %d", speedDouble, speed);
    

    // if((prevDir != reverse || abs(speed - prevSpeed) >= SPEED_DIFF) || !speed){
    //     if(prevSpeed == 0 && speed){
    //         set_mode(pigPio, MOTOR_ENABLE, PI_OUTPUT);
    //         set_mode(pigPio, MOTOR_P, PI_OUTPUT);
    //         set_mode(pigPio, MOTOR_M, PI_OUTPUT);
    //     }
    //     if(prevSpeed != 0 && speed == 0){
    //         carState = 0;
    //         set_mode(pigPio, MOTOR_ENABLE, PI_INPUT);
    //         set_mode(pigPio, MOTOR_P, PI_INPUT);
    //         set_mode(pigPio, MOTOR_M, PI_INPUT);
    //     }
    //     if(speed){
    //         gpio_write(pigPio, MOTOR_M, !reverse);
    //         gpio_write(pigPio, MOTOR_P, reverse);
    //         hardware_PWM(pigPio, MOTOR_ENABLE, FREQ_PWM, speed);
    //     }
    //     prevSpeed = speed;
    //     prevDir = reverse;
    // }
    prevSpeed = speed;

    return 0;
}

int car_rotate(double magnitude){
    if(magnitude < -SERVO_MAX_ANGLE){
        magnitude = -SERVO_MAX_ANGLE;
    }else if(magnitude > SERVO_MAX_ANGLE){
        magnitude = SERVO_MAX_ANGLE;
    }

    int pulseWidth = SERVO_MID + (int)round(magnitude*SERVO_RANGE/SERVO_MAX_ANGLE);
    // int pulseWidth =  (int)round((SERVO_MID + magnitude*SERVO_RANGE/SERVO_MAX_ANGLE)*SERVO_FREQ);
    // ROS_WARN("Pulse %lf %d\n", magnitude, pulseWidth);
    // int servo = SERVO_MID + SERVO_RANGE*magnitude/SERVO_IN_RANGE;
    // ROS_WARN("Angle %lf, %d", magnitude, pulseWidth);
    set_servo_pulsewidth(pigPio, SERVO_PIN, pulseWidth);
    // hardware_PWM(pigPio, SERVO_PIN, SERVO_FREQ, pulseWidth);

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
