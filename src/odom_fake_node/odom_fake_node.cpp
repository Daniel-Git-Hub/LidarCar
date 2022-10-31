#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "std_msgs/String.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpiod_if2.h>

#define ACC_X 1.0/1.0
#define ACC_HEADING 20.0


typedef struct {
    double velHeading;
    double velX;
    double velHeadingGoal;
    double velXGoal;

    double posHeading;
    double posX;
    double posY;
} odom_data_t;

ros::Time currentTime, lastTime;

ros::Duration waitTime(0, 1000*1000*100);

odom_data_t currentOdom = { 0 };

void twist_callback(const geometry_msgs::Twist& msg)
{
    currentOdom.velHeadingGoal = msg.angular.z;
    currentOdom.velXGoal       = msg.linear.x;
}


int main(int argc, char *argv[])
{
    printf("Start\n");
    ROS_INFO("Start ROS\n");
    ros::init(argc, argv, "odom_fake_node");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, twist_callback);

    // tf::TransformBroadcaster odom_broadcaster;

    
    // printf("dt, vH, pH, vX, vY, pX, pY\n");
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();
    while (ros::ok()) {
    // while (ros::ok()) {
        
        
        double deltaTime = (ros::Time::now() - currentTime).toSec();
        currentTime = ros::Time::now();
        
        currentOdom.velX = (currentOdom.velXGoal - currentOdom.velX)*deltaTime*ACC_X;
        currentOdom.velHeading = (currentOdom.velHeadingGoal - currentOdom.velHeading)*deltaTime*ACC_HEADING;

        
        if((ros::Time::now() - lastTime) > waitTime) {
            lastTime = ros::Time::now();
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
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = currentOdom.velHeading;
            // printf("Current Speed %lf %lf\n", currentOdom.velX, currentOdom.velHeading);
            //publish the message
            odom_pub.publish(odom);
            // comp_print();
        }
        ros::spinOnce();
    }    

    ROS_INFO("Close\n");
    return 0;
    // exit(0);
}

