/*This "topic_tool_platform.cpp" will receive "geometry_msgs::PoseStamped" type msgs,*/
/*The platform pose */
/*1:Receive the Position and Attitude*/
/*2:Calculate the velocity and angular rate*/
#include <iostream>
#include <cmath> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>
#include <string>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "filter.h"


using namespace Eigen;
using namespace std;







/*
Matrix<double, 3, 3> R;
Matrix<double, 3, 3> Rate_Change_Matrix;
Matrix<double, 3, 1> Euler_Rate_Change;
Matrix<double, 3, 1> Angular_Rate_Change;
*/
std_msgs::Float64MultiArray PLA_p;   //platform position
std_msgs::Float64MultiArray PLA_p_d; //platform velocity
std_msgs::Float64MultiArray PLA_r;   //platform attitude
std_msgs::Float64MultiArray PLA_agvr;//platform omga


std_msgs::Float64MultiArray Euler;   //platform attitude using for debug

geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped vel;
sensor_msgs::Imu imu;




void initialize(void);
void Position_and_Velocity(void);
void Attitude_and_Angular_rate(void);

/*now we just get cm */
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    pose = *msg;
    double roll, pitch, yaw;
    tf::Quaternion Q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
    Euler.data[0] = yaw;
    Euler.data[1] = pitch;
    Euler.data[2] = roll;

}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    vel = *msg;

}







int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_tool_platform");
    ros::NodeHandle nh;
    initialize();


    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/MAV5/mavros/local_position/pose", 10, pose_cb);
    
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/MAV5/mavros/local_position/velocity_local", 10,  vel_cb);
  

    /*the platform position*/
    ros::Publisher pos_pub =  nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/measure_position",10);

    /*the platform attitude*/
    ros::Publisher attitude_pub =  nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/measure_attitude",10);

    ros::Publisher attitude_euler_pub =  nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/measure_attitude_euler",10);

    /*the platform velocity*/
    ros::Publisher vel_pub =  nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/measure_velocity",10);

    /*the platform angular rate*/
    ros::Publisher omega_pub =  nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/measure_omega",10);

    
    ros::Rate rate(100);



    while (ros::ok()) {
      
        Position_and_Velocity();
        Attitude_and_Angular_rate();

        pos_pub.publish(PLA_p);
        vel_pub.publish(PLA_p_d);
        attitude_pub.publish(PLA_r);
        omega_pub.publish(PLA_agvr);
        attitude_euler_pub.publish(Euler);

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void initialize(void)
{
    PLA_p.data.resize(3); /*x,y,z*/
    PLA_p_d.data.resize(3); /*x,y,z*/
    PLA_r.data.resize(4); /*quaternion*/
    PLA_agvr.data.resize(3); /*Angular rate(not Euler angle rate change*/
    Euler.data.resize(3);
    

}


void Position_and_Velocity(void)
{
    
    PLA_p.data[0] = pose.pose.position.x;
    PLA_p.data[1] = pose.pose.position.y;
    PLA_p.data[2] = pose.pose.position.z;

    PLA_p_d.data[0] = vel.twist.linear.x;
    PLA_p_d.data[1] = vel.twist.linear.y;
    PLA_p_d.data[2] = vel.twist.linear.z;

}



void Attitude_and_Angular_rate(void)
{
   
   // quaternion.normalize();
    PLA_r.data[0] = pose.pose.orientation.w;
    PLA_r.data[1] = pose.pose.orientation.x;
    PLA_r.data[2] = pose.pose.orientation.y;
    PLA_r.data[3] = pose.pose.orientation.z;

    //transfer the attitude from Orientation to Euler
    PLA_agvr.data[0] = vel.twist.angular.x;
    PLA_agvr.data[1] = vel.twist.angular.y;
    PLA_agvr.data[2] = vel.twist.angular.z;



}
