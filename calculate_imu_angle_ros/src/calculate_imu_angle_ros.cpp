#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <cmath>

using namespace std;
#define PI 3.1415926

void imu1_Callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    // Extract angular velocity and linear acceleration from imu_msg
    tf::Vector3 angular_velocity(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    tf::Vector3 linear_acceleration(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

    double roll = sin(linear_acceleration.x() / 9.8) * 180 / PI;
    double pitch = sin(linear_acceleration.y() / 9.8) * 180 / PI + 45;
    // Print the rotation angles
    ROS_INFO_STREAM("IMU1: Roll: " << roll << "  |  Pitch: " << pitch << endl);
}
void imu2_Callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    // Extract orientation information from imu_msg
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_msg->orientation, quat);

    // Calculate roll, pitch, and yaw angles
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Print the rotation angles
    ROS_INFO("IMU2: Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "imu_reader");
    ros::NodeHandle nh;

    // Create a subscriber for IMU data
    ros::Subscriber imu1_sub = nh.subscribe("/camera/imu", 1, imu1_Callback);
    ros::Subscriber imu2_sub = nh.subscribe("/imu2_topic", 1, imu2_Callback);

    // Spin and process ROS callbacks
    ros::spin();

    return 0;
}