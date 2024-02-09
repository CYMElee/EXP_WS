#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "constant.h"
#include <Eigen/Dense>


std_msgs::Float32MultiArray desire_thrust;
std_msgs::Float32MultiArray desire_moment_attitude;
std_msgs::Float32MultiArray desire_moment_angle;

void desire_thrust_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    desire_thrust.data[0] = msg->data[0];
    desire_thrust.data[1] = msg->data[1];
    desire_thrust.data[2] = msg->data[2];

}

void desire_moment_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    desire_moment_attitude.data[0] = msg->data[0];
    desire_moment_attitude.data[1] = msg->data[1];
    desire_moment_attitude.data[2] = msg->data[2];
}
void desire_moment_angle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    desire_moment_angle.data[0] = msg->data[0];
    desire_moment_angle.data[1] = msg->data[1];
    desire_moment_angle.data[2] = msg->data[2];
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"wrench_mapper");
    ros::NodeHandle nh;

    ros::Subscriber phi_measure = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_thrust",10,desire_thrust_cb); //corresponding u1 on matlab

    ros::Subscriber phi_measure = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_moment_attitude",10,desire_moment_attitude_cb); // corresponding u2 on matlab


    ros::Subscriber phi_measure = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_moment_angle",10,desire_moment_angle_cb); // corresponding M on matlab

    while(ros::ok())
    {





    }

    return 0;
}
