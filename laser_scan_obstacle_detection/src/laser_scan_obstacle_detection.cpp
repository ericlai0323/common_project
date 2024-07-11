// src/laser_scan_obstacle_detection.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_robot_pub;
geometry_msgs::Twist current_cmd_vel;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (scan->ranges[i] < min_distance)
        {
            min_distance = scan->ranges[i];
        }
    }
    ROS_INFO("Closest obstacle distance: %f", min_distance);

    geometry_msgs::Twist cmd_vel_robot;
    if (min_distance <= 0.5)
    {
        cmd_vel_robot.linear.x = 0.0;
        cmd_vel_robot.linear.y = 0.0;
        cmd_vel_robot.linear.z = 0.0;
        cmd_vel_robot.angular.x = 0.0;
        cmd_vel_robot.angular.y = 0.0;
        cmd_vel_robot.angular.z = 0.0;
    }
    else
    {
        cmd_vel_robot = current_cmd_vel;
    }
    cmd_vel_robot_pub.publish(cmd_vel_robot);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    current_cmd_vel = *cmd_vel;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_obstacle_detection");
    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelCallback);
    cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_robot", 1000);

    ros::spin();
    return 0;
}
