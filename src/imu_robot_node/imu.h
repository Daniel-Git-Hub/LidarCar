#ifndef IMU_H_
#define IMU_H_

#include <stdlib.h>
#include <stdio.h>

typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t temp;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    double tranAccelX;
    double tranAccelY;
    double tranAccelZ;
    double tranGyroX;
    double tranGyroY;
    double tranGyroZ;
} imu_reading_t;

int imu_init(int);
imu_reading_t * imu_read();
int imu_close();
int imu_set_accel_scale(int);
int imu_set_gyro_scale(int);
int imu_print(int);
int imu_calibrate(int);

#endif