#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <math.h>
#include <iostream>

#define Kp 0.1
#define Ki 0
#define Kd 0.1



void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
    ROS_INFO_STREAM("I heard: [%s]" << vel_cmd.linear.y);
    std::cout << "Twist Received " << endl;
}

int main( int argc, char* argv[] )
{
    rROS_INFO("start");
    ros::init(argc, argv,"PID");
    ros::NodeHandle n;
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Publisher odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber sub = n.subscribe("odom", 1000, odometryCallback);
    ros::Rate loop_rate(10);

    while( n.ok() )
    {
        ros::spin();
    }

    return 0;
}
