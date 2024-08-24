#include "ros/ros.h"

#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define dt 0.01

float vel;
float vel_prev;

double roll, pitch, yaw;
double roll_prev, pitch_prev, yaw_prev;

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped pose_prev;

tf2::Quaternion quat;

std_msgs::Int32MultiArray position;
std_msgs::Int32MultiArray velocity;
std_msgs::Int32MultiArray attitude;
std_msgs::Int32MultiArray omega;



void platform_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;

}

void position()
{
    position[0] = pose.pose.position.x;
    position[1] = pose.pose.position.y;
    position[2] = pose.pose.position.z;
    platform_position.publish(position);
}


void velocity()
{    
    velocity[0] = (pose.pose.position.x - pose_prev.pose.position.x)/dt;
    velocity[1] = (pose.pose.position.y - pose_prev.pose.position.y)/dt;
    velocity[2] = (pose.pose.position.z - pose_prev.pose.position.z)/dt;
    platform_velocity.publish(velocity);

}

void attitude()
{
    
    tf2::fromMsg(pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    attitude[0] = roll;
    attitude[1] = pitch;
    attitude[2] = yaw;
    platform_attitude.publish(attitude);

}


void omega()
{
    tf2::fromMsg(pose_prev.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll_prev, pitch_prev, yaw_prev);
    omega[0] = (roll - roll_prev) / dt;
    omega[1] = (pitch - pitch_prev) / dt;
    omega[2] = (yaw - yaw_prev) / dt;
    platform_omega.publish(omega);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"platform_position");
    ros::NodeHandle nh;
    ros::Subscriber platform_pose = nh.subscribe<geometry_msgs::PoseStamped>
        ("/vrpn_client_node/Platform/pose",10,platform_pose_cb);
    ros::Publisher platform_position = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_position",10);
    ros::Publisher platform_velocity = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_velocity",10);
    ros::Publisher platform_attitude = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_attitude",10);
    ros::Publisher platform_omega = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_omega",10);


    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        position();
        velocity();
        attitude();
        omega();
        pose_prev = pose;
        rate.sleep();
    }

    return 0;
}