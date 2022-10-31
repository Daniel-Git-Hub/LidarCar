#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "std_msgs/String.h"

#include <stdlib.h>
#include <stdio.h>
// #include <iostream>

#include <pigpiod_if2.h>
#include "diff_encoder.h"

/*

REQUIRES

A rotary encoder contacts A and B connected to separate gpios and
the common contact connected to Pi ground.

TO BUILD

gcc -o rot_enc_cpp *.cpp -lpigpiod_if2 -lrt -lm

TO RUN

sudo ./rot_enc_cpp

gcc -o rot_enc_cpp *.cpp -lpigpiod_if2 -lrt -lm && ./rot_enc_cpp

*/

typedef struct {
    double velHeading;
    double velX;

    double posHeading;
    double posX;
    double posY;
} odom_data_t;

ros::Time rosLastTime;
ros::Duration waitTime(0, 1000*1000*100);

diff_encoder encoderController(HALF_BASE_WIDTH, ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B);

odom_data_t currentOdom = { 0 };

int pigPio = 0;

int main(int argc, char *argv[])
{
    printf("Start\n");
    ROS_INFO("Start ROS\n");
    ros::init(argc, argv, "diff_encoder_node");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    tf::TransformBroadcaster odom_broadcaster;


    pigPio = pigpio_start(NULL, NULL);
    if(pigPio < 0){
        printf("Failed Init\n");
        return 1;
    }
    
    encoderController.init(pigPio);
   

    rosLastTime = ros::Time::now();

    while(ros::ok()){
        ros::Time timeNow = ros::Time::now(); 
        if((timeNow - rosLastTime) > waitTime) {
        
            double dt = (timeNow - rosLastTime).toSec();
            
            encoderController.update(dt);
            speed_t * speed = encoderController.get_speed();

            currentOdom.velHeading = speed->angular;
            currentOdom.velX = speed->linear;

            currentOdom.posHeading += currentOdom.velHeading*dt;
            currentOdom.posX += cos(currentOdom.posHeading)*currentOdom.velX*dt;
            currentOdom.posY += sin(currentOdom.posHeading)*currentOdom.velX*dt;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(currentOdom.posHeading);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = timeNow;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = currentOdom.posX;
            odom_trans.transform.translation.y = currentOdom.posY;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // send the transform
            odom_broadcaster.sendTransform(odom_trans);

            nav_msgs::Odometry odom;
            odom.header.stamp = timeNow;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = currentOdom.posX;
            odom.pose.pose.position.y = currentOdom.posY;

            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = currentOdom.velX;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = currentOdom.velHeading;
            //publish the message
            odom_pub.publish(odom);

            // printf("Time %lf, ", time_time() - timeStart);
            // printf("Pos %d - %d, ", encoderController.leftEnc.pos, encoderController.rightEnc.pos);
            // printf("Speed %lf - %lf\n", speed->linear, speed->angular);
            // printf(", Speed %lf - %lf", leftEnc.speed, rightEnc.speed);

            rosLastTime = timeNow;

        }
        ros::spinOnce();
    }
   

   pigpio_stop(pigPio);
}



