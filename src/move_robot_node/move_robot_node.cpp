#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpiod_if2.h>
#include "car_motor.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void move_callback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Speed: [%s]", msg->data.c_str());
//     int speed;
//     int test = sscanf(msg->data.c_str(), "%d", &speed);
//     if(test == 1){
//         car_move(speed);
//     }
// }
// void rotate_callback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Rotate: [%s]", msg->data.c_str());
//     int rotate;
//     int test = sscanf(msg->data.c_str(), "%d", &rotate);
//     if(test == 1){
//         car_rotate(rotate);
//     }
// }

void twist_callback(const geometry_msgs::Twist& msg)
{
    ROS_INFO("MOVE_COMMAND: angle=[%lf], linear=[%lf]", msg.angular.z, msg.linear.x);
    car_rotate(msg.angular.z);
    car_move(msg.linear.x);
}

int main(int argc, char *argv[])
{
    ROS_INFO("Start ROS\n");
    ros::init(argc, argv, "move_robot_node");

    ros::NodeHandle n;
    // ros::Subscriber rotateSub = n.subscribe("rotate", 1000, rotate_callback);
    // ros::Subscriber moveSub = n.subscribe("move", 1000, move_callback);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, twist_callback);

    ROS_INFO("GPIO Init\n");
    int pigPio = pigpio_start(NULL, NULL);
    if (pigPio < 0) {
        ROS_ERROR("GPIO Init Failed\n");
        return -1;
    }
    
    car_init(pigPio);
    
    while (ros::ok()) {
        ros::spinOnce();
    }    
    // ROS_INFO("-100\n");
    // car_rotate(-100);
    
    // usleep(5000*1000);
    
    // ROS_INFO("0\n");
    // car_rotate(0);

    // usleep(5000*1000);

    // ROS_INFO("100\n");
    // car_rotate(100);

    // usleep(5000*2000);

    // ROS_INFO("0\n");
    // car_rotate(0);

    car_close();
    pigpio_stop(pigPio);
    ROS_INFO("Close\n");
    return 0;
    // exit(0);
}

