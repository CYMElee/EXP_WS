#include "ros/ros.h"
#include ""



geometry_msgs::PoseStamped system_pose;  //position
geometry_msgs::PoseStamped system_vel; //velocity

std_msgs::Float32MultiArray gripper;
std_msgs::Float32 phid;   //angle
std_msgs::Float32 phid_d; //angular vel
std_msgs::Float32 phid_dd; //angular acc