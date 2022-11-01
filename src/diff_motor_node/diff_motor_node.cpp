#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"


#include <stdlib.h>
#include <stdio.h>
// #include <iostream>

#include <pigpiod_if2.h>
#include "diff_motor.h"


diff_motor motorController(HALF_BASE_WIDTH, MOTOR_LEFT_P, MOTOR_LEFT_M, MOTOR_LEFT_EN, MOTOR_RIGHT_P, MOTOR_RIGHT_M, MOTOR_RIGHT_EN);

int pigPio = 0;


void twist_callback(const geometry_msgs::Twist& msg)
{
    ROS_INFO("MOVE_COMMAND: angle=[%lf], linear=[%lf]", msg.angular.z, msg.linear.x);
    // speed_t speed = { msg.linear.x, msg.angular.z };
    motorController.set_speed(msg.linear.x, msg.angular.z);
}


int main(int argc, char *argv[])
{
    ROS_INFO("Start ROS\n");
    ros::init(argc, argv, "diff_motor_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, twist_callback);


    pigPio = pigpio_start(NULL, NULL);
    if(pigPio < 0){
        ROS_ERROR("DIFF Failed Init\n");
        return 1;
    }
    motorController.init(pigPio);

    while(ros::ok()){
        ros::spinOnce();
    }

    motorController.close();

    pigpio_stop(pigPio);
}



