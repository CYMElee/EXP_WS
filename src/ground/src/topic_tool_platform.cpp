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
#include "Eigen/Dense"
#include "filter.h"

#define dt 0.01

using namespace Eigen;
using namespace std;

float Psi = 0; // x
float Theta = 0; // y
float Phi = 0; // z

int t = 0;

geometry_msgs::PoseStamped host_mocap;
geometry_msgs::Vector3Stamped imu_rate;

Vector3d a; //a & b using for filter
Vector4d b;

Vector3d c;
Vector4d d;

Vector3d e;
Vector4d f;


Vector3d p;
Vector3d p_prev;


Vector4d q;
Vector3d eu;

Vector3d o;

Matrix<double, 3, 3> R;
Matrix<double, 3, 3> Rate_Change_Matrix;
Matrix<double, 3, 1> Euler_Rate_Change;
Matrix<double, 3, 1> Angular_Rate_Change;

std_msgs::Float64MultiArray PLA_p;   //platform position
std_msgs::Float64MultiArray PLA_p_d; //platform velocity
std_msgs::Float64MultiArray PLA_r;   //platform attitude
std_msgs::Float64MultiArray PLA_agvr;//platform omga


std_msgs::Float64MultiArray Euler;   //platform attitude using for debug

geometry_msgs::PoseStamped pose;






void initialize(void);
void Position_and_Velocity(void);
void Attitude_and_Angular_rate(void);

/*now we just get cm */
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   pose = *msg;
    pose.pose.position.x = std::floor(pose.pose.position.x*1000)/1000;
    pose.pose.position.y = std::floor(pose.pose.position.y*1000)/1000;
    pose.pose.position.z = std::floor(pose.pose.position.z*1000)/1000;
    pose.pose.orientation.w = std::floor(pose.pose.orientation.w*1000)/1000;
    pose.pose.orientation.x = std::floor(pose.pose.orientation.x*1000)/1000;
    pose.pose.orientation.y = std::floor(pose.pose.orientation.y*1000)/1000;
    pose.pose.orientation.z = std::floor(pose.pose.orientation.z*1000)/1000;
}

void imu_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    imu_rate = *msg;
    imu_rate.vector.x = std::floor(imu_rate.vector.x*1000)/1000;
    imu_rate.vector.y = std::floor(imu_rate.vector.y*1000)/1000;
    imu_rate.vector.z = std::floor(imu_rate.vector.z*1000)/1000;
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_tool_platform");
    ros::NodeHandle nh;
    initialize();
    a << -0.726542528005361,0,0;
    b <<0.136728735997320,0.136728735997320,0,0;

    c << -1.911197067426073,0.914975834801434,0;
    d <<9.446918438401550e-04,0.001889383687680,9.446918438401550e-04,0;

    e<< -1.911197067426073,0.914975834801434,0;
    f<< 9.446918438401550e-04,0.001889383687680,9.446918438401550e-04,0;
    
    filter position_filter(a,b);
    filter attitude_filter(c,d);
    filter angular_rate_filter(e,f);

    std::string sub_topic = std::string("/vrpn_client_node/platform") + std::string("/pose");

    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, host_pos);

    /*get the angular velocity from MTI-300 IMU*/
    ros::Subscriber imu_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/imu/angular_velocity",10,imu_cb);

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

    for(int i = 0;i<50; i++){
        /*set the position_prev*/
        p_prev(0) = pose.pose.position.x;
        p_prev(1) = pose.pose.position.y;
        p_prev(2) = pose.pose.position.z;
        /*set the */
  
        ros::spinOnce();
        rate.sleep();
    }


    while (ros::ok()) {

        p = position_filter.Butterworth_filter_position(pose,t);
        q = attitude_filter.Butterworth_filter_attitude(pose,t);
        o = angular_rate_filter.Butterworth_filter_angular_rate(imu_rate,t);
        
        Position_and_Velocity();
        Attitude_and_Angular_rate();

        pos_pub.publish(PLA_p);
        vel_pub.publish(PLA_p_d);
        attitude_pub.publish(PLA_r);
        omega_pub.publish(PLA_agvr);
        attitude_euler_pub.publish(Euler);

        p_prev = p;
        
        ros::spinOnce();
        rate.sleep();
        t++;
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
    



    /*set the init position using to get init velocity*/


}


void Position_and_Velocity(void)
{
    
    PLA_p.data[0] = p(0);
    PLA_p.data[1] = p(1);
    PLA_p.data[2] = p(2);

    PLA_p_d.data[0] = ((p(0)-p_prev(0))/dt);
    PLA_p_d.data[1] = ((p(1)-p_prev(1))/dt);
    PLA_p_d.data[2] = ((p(2)-p_prev(2))/dt);

}



void Attitude_and_Angular_rate(void)
{
    Quaterniond quaternion(q(0),q(1),q(2),q(3));
   // quaternion.normalize();
    PLA_r.data[0] = q(0);
    PLA_r.data[1] = q(1);
    PLA_r.data[2] = q(2);
    PLA_r.data[3] = q(3);

    //transfer the attitude from Orientation to Euler
    PLA_agvr.data[0] =  o(0);
    PLA_agvr.data[1] =  o(1);
    PLA_agvr.data[2] =  o(2);

}
