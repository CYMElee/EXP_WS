#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include "Eigen/Dense"

#define gravity 9.81 
#define  Mass 0.5 //kg


using namespace Eigen;

std_msgs::Float32MultiArray t;

RowVector3f p;
RowVector3f pd;
RowVector3f p_dot;
RowVector3f pd_dot;
RowVector3f fr;
RowVector3f u1;


Matrix<float, 3, 3> Kp;
Matrix<float, 3, 3> Kv;
Matrix<float, 3, 3> Ki;
Matrix<float, 3, 3> R;

Matrix<float, 3, 1> ep;
Matrix<float, 3, 1> ev;
Matrix<float, 3, 1> fr;
Matrix<float, 3, 1> z; //0,0,1


//control gain
Kp << 4.5, 0 , 0 ,
      0  , 4.5,0
      0  , 0,  5;

Kv << 2  , 0  ,0 ,
      0  , 2  ,0,
      0  , 0  ,2.5;

Ki << 2.7, 0 , 0 ,
      0  , 2.5,0,
      0  , 0,  3;
z<< 0,0,1;


void desire_position_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    pd(0) = msg->data[0];
    pd(1)= msg->data[1];
    pd(2) = msg->data[2];

}
void desire_velocity_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    pd_dot(0) = msg->data[0];
    pd_dot(1) = msg->data[1];
    pd_dot(2) = msg->data[2];

}
void measure_position_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    p(0) = msg->data[0];
    p(1) = msg->data[1];
    p(2) = msg->data[2];

}
void measure_velocity_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    p_dot(0) = msg->data[0];
    p_dot(1) = msg->data[1];
    p_dot(2) = msg->data[2];
}

void measure_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Eigen::AngleAxisd roll_angle(msg ->data[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(msg ->data[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(msg ->data[2], Eigen::Vector3d::UnitZ());
    R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    
}

void total_thrust()
{
    ep(0) = p(0) - pd(0);
    ep(1) = p(1) - pd(1);
    ep(2) = p(2) - pd(2);

    ev(0) = p_dot(0) - p_dot(0);
    ev(1) = p_dot(1) - p_dot(1);
    ev(2) = p_dot(2) - p_dot(2);

    fr = Mass*(gravity*z-Kp*ep-Kv*ev);
    u1 = R.transpose*fr;

    t[0] = u1(0);
    t[1] = u1(1);
    t[2] = u1(2);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"position_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber desire_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_position",10,desire_position_cb); 

    ros::Subscriber desire_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_velocity",10,desire_velocity_cb);

    ros::Subscriber measure_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_position",10,measure_position_cb); 

    ros::Subscriber measure_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_velocity",10,measure_velocity_cb); 

    ros::Subscriber measure_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_attitude",10,measure_attitude_cb); 

    ros::Publisher desire_thrust_total = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/desire_thrust_total",10);

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        total_thrust();

        desire_thrust_total.publish();
    
        rate.sleep(t);

    }
    return 0;
}