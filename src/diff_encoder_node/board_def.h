#ifndef BOARD_DEF_H
#define BOARD_DEF_H

#define M_PER_ENCODER (2.0/5258.0)
#define ANUGULAR_GAIN (1000000/7.9)    //dutyCylce/(rad/s)
#define LINEAR_GAIN   (1/0.21)  //dutyCylce/(m/s)

#define LEFT_GAIN  1
#define RIGHT_GAIN 1
#define ANGLE_GAIN -0.1
#define ENCODER_TIME_UPDATE 0.025

#define PMW_FREQUENCY 40
#define PWM_MIN 400000
#define PWM_MAX 1000000

#define ENC_RIGHT_A 3
#define ENC_RIGHT_B 4

#define MOTOR_RIGHT_P 7
#define MOTOR_RIGHT_M 26
#define MOTOR_RIGHT_EN 19


#define ENC_LEFT_A 9
#define ENC_LEFT_B 11

#define MOTOR_LEFT_P 16
#define MOTOR_LEFT_M 20
#define MOTOR_LEFT_EN 12


#define HALF_BASE_WIDTH 0.060

typedef struct {
    double linear;
    double angular;
} speed_t;

#endif