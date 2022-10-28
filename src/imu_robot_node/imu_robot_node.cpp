#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "std_msgs/String.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpiod_if2.h>
#include "imu.h"
#include "comp.h"



typedef struct {
    double velHeading;
    double posHeading;

    double velX;
    double velY;
    
    double posX;
    double posY;
} odom_data_t;

ros::Time currentTime, lastTime;

ros::Duration waitTime(0, 1000*1000*500);

odom_data_t currentOdom = { 0 };


int main(int argc, char *argv[])
{
    printf("Start\n");
    ROS_INFO("Start ROS\n");
    ros::init(argc, argv, "imu_robot_node");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    // tf::TransformBroadcaster odom_broadcaster;


    ROS_INFO("IMU Init\n");
    int pigPio = pigpio_start(NULL, NULL);
    if (pigPio < 0) {
        ROS_ERROR("IMU Init Failed\n");
        return -1;
    }
    ROS_INFO("IMU Init Success\n");
    
    imu_init(pigPio);
    ROS_INFO("IMU Start Calibrate\n");
    imu_calibrate(20);
    ROS_INFO("IMU Stop Calibrate\n");

    // printf("dt, vH, pH, vX, vY, pX, pY\n");
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();
    while (ros::ok()) {
    // while (ros::ok()) {
        
        
        imu_reading_t * reading = imu_read();

        ros::Duration deltaTime = (ros::Time::now() - currentTime);
        currentTime = ros::Time::now();
        // comp_calc(deltaTime.toSec(), reading);

        // currentOdom.velHeading = (1-GAIN_VEL_H)*currentOdom.velHeading + GAIN_VEL_H*reading->tranGyroZ;
        currentOdom.velHeading += reading->tranGyroZ;

        // currentOdom.posHeading = (1-GAIN_POS_H)*currentOdom.posHeading + GAIN_POS_H*currentOdom.velHeading*deltaTime.toSec();
        currentOdom.posHeading = currentOdom.velHeading*deltaTime.toSec();

        
        // currentOdom.velX = (1-GAIN_VEL_XY)*currentOdom.velX + GAIN_VEL_XY*deltaTime.toSec()*reading->tranAccelX;
        // currentOdom.velY = (1-GAIN_VEL_XY)*currentOdom.velX + GAIN_VEL_XY*deltaTime.toSec()*reading->tranAccelY;        
        currentOdom.velX += deltaTime.toSec()*reading->tranAccelX; 
        currentOdom.velY += deltaTime.toSec()*reading->tranAccelY; 

        currentOdom.posX += cos(currentOdom.posHeading)*deltaTime.toSec()*currentOdom.velX + sin(currentOdom.posHeading)*deltaTime.toSec()*currentOdom.velY; 
        currentOdom.posY += sin(currentOdom.posHeading)*deltaTime.toSec()*currentOdom.velX + cos(currentOdom.posHeading)*deltaTime.toSec()*currentOdom.velY; 

        
        if((ros::Time::now() - lastTime) > waitTime) {
            lastTime = ros::Time::now();
            // imu_print(0);

            // printf("%le, %lf, %lf, %lf, %lf, %lf, %lf\n", deltaTime.toSec(), currentOdom.velHeading, currentOdom.posHeading, currentOdom.velX, currentOdom.velY, currentOdom.posX, currentOdom.posY);

            // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(currentOdom.posHeading);
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

            // geometry_msgs::TransformStamped odom_trans;
            // odom_trans.header.stamp = currentTime;
            // odom_trans.header.frame_id = "odom";
            // odom_trans.child_frame_id = "base_link";

            // odom_trans.transform.translation.x = 0;
            // // odom_trans.transform.translation.x = currentOdom.posX;
            // odom_trans.transform.translation.y = 0;
            // // odom_trans.transform.translation.y = currentOdom.posY;
            // odom_trans.transform.translation.z = 0.0;
            // odom_trans.transform.rotation = odom_quat;

            //send the transform
            // odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = lastTime;
            odom.header.frame_id = "odom";

            //set the position
            // odom.pose.pose.position.x = currentOdom.posX;
            odom.pose.pose.position.x = 0;
            // odom.pose.pose.position.y = currentOdom.posY;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = currentOdom.velX;
            odom.twist.twist.linear.y = currentOdom.velY;
            odom.twist.twist.angular.z = currentOdom.velHeading;

            //publish the message
            odom_pub.publish(odom);
            // comp_print();
        }
        ros::spinOnce();
    }    

    pigpio_stop(pigPio);
    printf("Close\n");
    ROS_INFO("Close\n");
    return 0;
    // exit(0);
}

