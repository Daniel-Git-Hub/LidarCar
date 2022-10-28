#ifndef COMP_H_
#define COMP_H_

#include "imu.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


tf::Quaternion * comp_calc(double dt, imu_reading_t * reading);
int comp_print();

#endif