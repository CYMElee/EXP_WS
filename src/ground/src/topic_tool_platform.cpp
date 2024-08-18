/*This "topic_tool_platform.cpp" will receive "geometry_msgs::PoseStamped" type msgs,*/
/*The platform pose */
/*1:Receive the Position and Attitude*/
/*2:Calculate the velocity and angular rate*/

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
#include "Eigen/Dense"

#define dt 0.01

using namespace Eigen;

float Psi = 0; // x
float Theta = 0; // y
float Phi = 0; // z

geometry_msgs::PoseStamped host_mocap;



RowVector3f p;
RowVector3f p_prev;

RowVector3f eu;
RowVector3f eu_prev;

Matrix<float, 3, 3> R;
Matrix<float, 3, 3> Rate_Change_Matrix;
Matrix<float, 3, 1> Euler_Rate_Change;
Matrix<float, 3, 1> Angular_Rate_Change;

std_msgs::Float32MultiArray PLA_p;   //platform position
std_msgs::Float32MultiArray PLA_p_d; //platform velocity
std_msgs::Float32MultiArray PLA_r;   //platform attitude
std_msgs::Float32MultiArray PLA_agvr;//platform omga

geometry_msgs::PoseStamped pose;




void initialize(void);
void Position_and_Velocity(void);
void Attitude_and_Angular_rate(void);


void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{  
    pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_tool_platform");
    ros::NodeHandle nh;

    std::string sub_topic = std::string("/vrpn_client_node/platform") + std::string("/pose");

    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, host_pos);

    /*the platform position*/
    ros::Publisher pos_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_position",10);

    /*the platform attitude*/
    ros::Publisher attitude_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_attitude",10);

    /*the platform velocity*/
    ros::Publisher vel_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10);

    /*the platform angular rate*/
    ros::Publisher omega_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_omega",10);

    
    ros::Rate rate(100);

    while (ros::ok()) {

        Position_and_Velocity();
        Attitude_and_Angular_rate();

        pos_pub.publish(PLA_p);
        vel_pub.publish(PLA_p_d);
        attitude_pub.publish(PLA_r);
        omega_pub.publish(PLA_agvr);

        p_prev = p;
        eu_prev = eu;
        

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


    /*set the init position using to get init velocity*/
    p_prev[0] = pose.pose.position.x;
    p_prev[1] = pose.pose.position.y;
    p_prev[2] = pose.pose.position.z;

}


void Position_and_Velocity(void)
{
    PLA_p.data[0] = pose.pose.position.x;
    PLA_p.data[1] = pose.pose.position.y;
    PLA_p.data[2] = pose.pose.position.z;
    p[0] = pose.pose.position.x;
    p[1] = pose.pose.position.y;
    p[2] = pose.pose.position.z;

    PLA_p_d.data[0] = (p[0]-p_prev[0])/dt;
    PLA_p_d.data[1] = (p[1]-p_prev[1])/dt;
    PLA_p_d.data[2] = (p[2]-p_prev[2])/dt;

}



void Attitude_and_Angular_rate(void)
{
    Quaternionf quaternion(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
    PLA_r.data[0] = pose.pose.orientation.w;
    PLA_r.data[1] = pose.pose.orientation.x;
    PLA_r.data[2] = pose.pose.orientation.y;
    PLA_r.data[3] = pose.pose.orientation.z;

    //transfer the attitude from Orientation to Euler
    
    R = quaternion.toRotationMatrix();
    eu = R.eulerAngles(2,1,0);
    Psi = eu(2); //x
    Theta = eu(1); //y
    Phi = eu(0); //z


    Rate_Change_Matrix <<1 ,0 ,-sin(Theta),
                          0,cos(Psi),sin(Psi)*cos(Theta),
                          0,-sin(Psi),cos(Psi)*cos(Theta);


    Euler_Rate_Change(0) = ((eu[2] - eu_prev[2])/dt);
    Euler_Rate_Change(1) = ((eu[1] - eu_prev[1])/dt);
    Euler_Rate_Change(2) = ((eu[0] - eu_prev[0])/dt);
    
    Angular_Rate_Change = Rate_Change_Matrix*Euler_Rate_Change;

    PLA_agvr.data[0] = Angular_Rate_Change(0);
    PLA_agvr.data[1] = Angular_Rate_Change(1);
    PLA_agvr.data[2] = Angular_Rate_Change(2);

}
