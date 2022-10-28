#include "imu.h"
// #include <ros/ros.h>

#include <cstring>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "imu_reg.h"
#include <math.h>

#define IMU_START() (imuHandle = i2c_open(imuPio, IMU_I2C_BUS, IMU_ADDR, IMU_I2C_FLAG))
#define IMU_STOP() (i2c_close(imuPio, imuHandle))
#define IMU_WRITE_BYTE(b) (i2c_write_byte(imuPio, imuHandle, b))

#define GRAV 9.807  

#define FLIP_XY -1

int imuPio = -1;

int imuHandle = -1;


int accelScale = IMU_ACCEL_2G;
int gyroScale = IMU_GYRO_250;
double gyroCo = M_PI * 250.0 * (1 << IMU_GYRO_250) / (32768.0 * 180.0);
double accelCo = GRAV * (2 << IMU_ACCEL_2G) / 32768.0;

imu_reading_t lastReading = { 0 };
imu_reading_t calibrateReading = { 0 };

int imu_init(int p){
    imuPio = p;
    
    imuHandle = i2c_open(imuPio, IMU_I2C_BUS, IMU_ADDR, IMU_I2C_FLAG);
    if(imuHandle < 0){
        // ROS_ERROR("start failure\n");
        printf("start failure\n");
        return -1;
    }
    
    int result = i2c_write_byte_data(imuPio, imuHandle, 0x6B, 0);
    if(result < 0){
        // ROS_ERROR("init bytes failure\n");
        printf("init bytes failure\n");
        return -1;
    }
    
    result = i2c_close(imuPio, imuHandle);
    if(result < 0){
        // ROS_ERROR("stop failure\n");
        printf("stop failure\n");
        return -1;
    }

    imu_set_accel_scale(IMU_ACCEL_2G);
    imu_set_gyro_scale(IMU_GYRO_250);

    return 0;
}

imu_reading_t * imu_read(){
    char reading[14] = { 0 };

    imuHandle = i2c_open(imuPio, IMU_I2C_BUS, IMU_ADDR, IMU_I2C_FLAG);
    if(imuHandle < 0){
        return 0;
    }
    // lastReading.accelZ = ((i2c_read_byte_data(imuPio, IMU_I2C_HANDLE, 0x3F) << 8) & 0xff00) | (i2c_read_byte_data(imuPio, IMU_I2C_HANDLE, 0x40) & 0xff);
    // printf("Reading %d - %lf\n", lastReading.accelZ, lastReading.accelZ*accelCo);

    int result = i2c_read_i2c_block_data(imuPio, imuHandle, 0x3B, reading, 14);
    if(result < 0){
        return 0;
    }

    lastReading.accelX = ((reading[0] << 8) & 0xff00)  | (reading[1] & 0xff);
    lastReading.accelY = ((reading[2] << 8) & 0xff00)  | (reading[3] & 0xff);
    lastReading.accelZ = ((reading[4] << 8) & 0xff00)  | (reading[5] & 0xff);

    lastReading.temp   = ((reading[6] << 8) & 0xff00)  | (reading[7] & 0xff);

    lastReading.gyroX  = ((reading[8] << 8) & 0xff00)  | (reading[9] & 0xff);
    lastReading.gyroY  = ((reading[10] << 8) & 0xff00) | (reading[11] & 0xff);
    lastReading.gyroZ  = ((reading[12] << 8) & 0xff00) | (reading[13] & 0xff);

    result = i2c_close(imuPio, imuHandle);
    if(result < 0){
        return 0;
    }

    lastReading.tranAccelX = FLIP_XY * lastReading.accelX * accelCo - calibrateReading.tranAccelX;
    lastReading.tranAccelY = FLIP_XY * lastReading.accelY * accelCo - calibrateReading.tranAccelY;
    lastReading.tranAccelZ = lastReading.accelZ * accelCo - calibrateReading.tranAccelZ;

    lastReading.tranGyroX = lastReading.gyroX * gyroCo - calibrateReading.tranGyroX;
    lastReading.tranGyroY = lastReading.gyroY * gyroCo - calibrateReading.tranGyroY;
    lastReading.tranGyroZ = lastReading.gyroZ * gyroCo - calibrateReading.tranGyroZ;
    

    return &lastReading;
}

int imu_calibrate(int count){
    // memset(&calibrateReading, 0x00, sizeof(imu_reading_t));
    
    imu_reading_t temp = { 0 };
    
    for(int i = 0; i < count; i++){
        if(!imu_read()){
            // ROS_ERROR("Reading failed\n");
            printf("Reading failed\n");
        }
        temp.tranAccelX += lastReading.tranAccelX;
        temp.tranAccelY += lastReading.tranAccelY;
        temp.tranAccelZ += lastReading.tranAccelZ;
        
        temp.tranGyroX  += lastReading.tranGyroX;
        temp.tranGyroY  += lastReading.tranGyroY;
        temp.tranGyroZ  += lastReading.tranGyroZ;
        
        // printf("Read %lf %lf %lf - ", lastReading.tranAccelX, lastReading.tranAccelY, lastReading.tranAccelZ);
        // printf("%lf %lf %lf\n", lastReading.tranGyroX, lastReading.tranGyroY, lastReading.tranGyroZ);


        usleep(1000*100);
    }
    
    memcpy(&calibrateReading, &temp, sizeof(imu_reading_t));

    // calibrateReading.tranAccelX /= count;
    // calibrateReading.tranAccelY /= count;
    // calibrateReading.tranAccelZ /= count;

    calibrateReading.tranGyroX  /= count;
    calibrateReading.tranGyroY  /= count;
    calibrateReading.tranGyroZ  /= count;
    // printf("Cal %lf %lf %lf - ", calibrateReading.tranAccelX, calibrateReading.tranAccelY, calibrateReading.tranAccelZ);
    // printf("%lf %lf %lf\n", calibrateReading.tranGyroX, calibrateReading.tranGyroY, calibrateReading.tranGyroZ);
    return 0;
}

int imu_print(int toRead){
    if(toRead){
        if(!imu_read()){
            // ROS_ERROR("IMU Reading failed\n");
            printf("IMU Reading failed\n");
            return -1;
        }
    }
    // ROS_INFO("Accel %lf %lf %lf - ", lastReading.tranAccelX, lastReading.tranAccelY, lastReading.tranAccelZ);
    printf("Accel %lf %lf %lf - ", lastReading.tranAccelX, lastReading.tranAccelY, lastReading.tranAccelZ);
    // ROS_INFO("Gyro %lf %lf %lf\n", lastReading.tranGyroX, lastReading.tranGyroY, lastReading.tranGyroZ);
    printf("Gyro %lf %lf %lf\n", lastReading.tranGyroX, lastReading.tranGyroY, lastReading.tranGyroZ);
    return 0;
}

int imu_close(){
    IMU_STOP();
    if(imuHandle >= 0){
        return i2c_close(imuPio, imuHandle);
    }
    return 0;
}

int imu_set_accel_scale(int scale){
    accelScale = scale;

    imuHandle = i2c_open(imuPio, IMU_I2C_BUS, IMU_ADDR, IMU_I2C_FLAG);

    int result = i2c_write_byte_data(imuPio, imuHandle, 0x1C, accelScale << 3);

    result = i2c_close(imuPio, imuHandle);

    accelCo = GRAV * (2 << accelScale) / 32768.0;
    return 0;
}

int imu_set_gyro_scale(int scale){
    gyroScale = scale;
    imuHandle = i2c_open(imuPio, IMU_I2C_BUS, IMU_ADDR, IMU_I2C_FLAG);

    int result = i2c_write_byte_data(imuPio, imuHandle, 0x1B, gyroScale << 3);
    // IMU_WRITE_BYTE(0x1B);
    // IMU_WRITE_BYTE(gyroScale << 3);
    result = i2c_close(imuPio, imuHandle);
    gyroCo = M_PI * 250.0 * (1 << IMU_GYRO_250) / (32768.0 * 180.0);
    // gyroCo = 250.0*(1 << gyroScale) / 32768.0;
    return 0;
}