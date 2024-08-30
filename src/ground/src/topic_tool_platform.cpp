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
#include "filter.h"

#define dt 0.01

using namespace Eigen;

float Psi = 0; // x
float Theta = 0; // y
float Phi = 0; // z

int t = 0;

geometry_msgs::PoseStamped host_mocap;

Vector3d a; //a & b using for filter
Vector4d b;

Vector3d c;
Vector4d d;


Vector3d p;
Vector3d p_prev;


Vector4d q;
Vector3d eu;
Vector3d eu_prev;

Matrix<double, 3, 3> R;
Matrix<double, 3, 3> Rate_Change_Matrix;
Matrix<double, 3, 1> Euler_Rate_Change;
Matrix<double, 3, 1> Angular_Rate_Change;

std_msgs::Float32MultiArray PLA_p;   //platform position
std_msgs::Float32MultiArray PLA_p_d; //platform velocity
std_msgs::Float32MultiArray PLA_r;   //platform attitude
std_msgs::Float32MultiArray PLA_agvr;//platform omga


std_msgs::Float32MultiArray Euler;   //platform attitude using for debug

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
    initialize();
    a << -2.374094743709352,1.929355669091215,-0.532075368312092;
    b <<0.002898194633721,0.008694583901164,0.008694583901164,0.002898194633721;

    c << -2.874356892677485,2.756483195225695,-0.881893130592486;
    d << 2.914649446567053e-05,8.743948339701157e-05,8.743948339701157e-05,2.914649446567053e-05;
    
    filter position_filter(a,b);
    filter attitude_filter(c,d);

    std::string sub_topic = std::string("/vrpn_client_node/platform") + std::string("/pose");

    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, host_pos);

    /*the platform position*/
    ros::Publisher pos_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_position",10);

    /*the platform attitude*/
    ros::Publisher attitude_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_attitude",10);

    ros::Publisher attitude_euler_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_attitude_euler",10);

    /*the platform velocity*/
    ros::Publisher vel_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10);

    /*the platform angular rate*/
    ros::Publisher omega_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_omega",10);

    
    ros::Rate rate(100);

    for(int i = 0;i<50; i++){
        /*set the position_prev*/
        p_prev(0) = pose.pose.position.x;
        p_prev(1) = pose.pose.position.y;
        p_prev(2) = pose.pose.position.z;
        /*set the */
        eu_prev(2) = 0;
        eu_prev(1) = 0;
        eu_prev(0) = 0;
        ros::spinOnce();
        rate.sleep();
    }


    while (ros::ok()) {

        p = position_filter.Butterworth_filter_position(pose,t);
        q = attitude_filter.Butterworth_filter_attitude(pose,t);
        Position_and_Velocity();
        Attitude_and_Angular_rate();

        pos_pub.publish(PLA_p);
        vel_pub.publish(PLA_p_d);
        attitude_pub.publish(PLA_r);
        omega_pub.publish(PLA_agvr);
        attitude_euler_pub.publish(Euler);

        p_prev = p;
        eu_prev = eu;
        

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
    PLA_r.data[0] = q(0);
    PLA_r.data[1] = q(1);
    PLA_r.data[2] = q(2);
    PLA_r.data[3] = q(3);

    //transfer the attitude from Orientation to Euler
    
    R = quaternion.toRotationMatrix();
    eu = R.eulerAngles(2,1,0);

    Euler.data[0] = eu(0);
    Euler.data[1] = eu(1);
    Euler.data[2] = eu(2);

    Psi = eu(2); //x
    Theta = eu(1); //y
    Phi = eu(0); //z


    Rate_Change_Matrix <<1 ,0 ,-sin(Theta),
                          0,cos(Psi),sin(Psi)*cos(Theta),
                          0,-sin(Psi),cos(Psi)*cos(Theta);


    Euler_Rate_Change(0) = ((eu(2) - eu_prev(2))/dt);
    Euler_Rate_Change(1) = ((eu(1) - eu_prev(1))/dt);
    Euler_Rate_Change(2) = ((eu(0) - eu_prev(0))/dt);
    
    Angular_Rate_Change = Rate_Change_Matrix*Euler_Rate_Change;

    PLA_agvr.data[0] = Angular_Rate_Change(0);
    PLA_agvr.data[1] = Angular_Rate_Change(1);
    PLA_agvr.data[2] = Angular_Rate_Change(2);

}
