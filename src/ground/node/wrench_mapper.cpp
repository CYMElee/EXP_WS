#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <Eigen/Dense>
#define lw  0.8; //units: meter

using namespace Eigen;

std_msgs::Float64MultiArray u1;
std_msgs::Float64MultiArray u2;
std_msgs::Float64 M;
std_msgs::Float64MultiArray Fd;


Vector3d BP1;
Vector3d BP2;
Vector3d BP3;
Vector3d BP4;

Matrix<double, 3, 3> BP1x;
Matrix<double, 3, 3> BP2x;
Matrix<double, 3, 3> BP3x;
Matrix<double, 3, 3> BP4x;

Matrix<double, 3, 3> B1R;
Matrix<double, 3, 3> B2R;
Matrix<double, 3, 3> B3R;
Matrix<double, 3, 3> B4R;


Matrix<double, 6, 12> A;
Matrix<double, 12,12> Ar_temp1;
Matrix<double, 6,12> Ar_temp2;
Matrix<double, 7,12> Ar;
Matrix<double, 12,7> ArT;
Matrix<double, 7, 1> fd_temp1;
Matrix<double, 12, 1> fd;
RowVector3d W1;
RowVector3d W2;
RowVector3d W3;
RowVector3d W4;

float Lw = lw;

void desire_thrust_total_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    u1 = *msg;

}

void desire_moment_attitude_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    u2 = *msg;
}
void desire_moment_angle_cb(const std_msgs::Float64::ConstPtr& msg)
{
    M = *msg;
    
}

void fd_gen()
{
    for(int r = 0; r< 3;r++)
    {
    fd_temp1(r,0) = u1.data[r];  
    }
    for(int r = 0; r< 3;r++)
    {
    fd_temp1(r+3,0) = u2.data[r];
    }
    fd_temp1(6,0) = M.data;
    fd = ArT* fd_temp1;
    for(int r = 0;r<12;r++)
    {
        Fd.data[r] = fd(r,0);
        
    }

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"wrench_mapper");
    ros::NodeHandle nh;
    BP1 << Lw , 0, 0 ;
    BP2 << 0, Lw , 0 ;
    BP3 << -Lw , 0, 0 ;
    BP4 << 0, -Lw , 0 ;

    BP1x << 0, -BP1(2),BP1(1),
            BP1(2),0,-BP1(0),
            -BP1(1),BP1(0),0;

    BP2x << 0,-BP2(2),BP2(1), 
            BP2(2),0,-BP2(0), 
            -BP2(1),BP2(0),0;

    BP3x << 0,-BP3(2),BP3(1),
            BP3(2),0,-BP3(0),
            -BP3(1),BP3(0),0;

    BP4x << 0,-BP4(2),BP4(1), 
            BP4(2),0,-BP4(0),
            -BP4(1),BP4(0),0;
    B1R = Matrix3d::Identity();
    B2R <<  0 ,-1  , 0,
            1 , 0  , 0,
            0 , 0  , 1;
    B3R << -1 , 0  , 0,
            0 ,-1  , 0,
            0 , 0  , 1;  
    B4R <<  0 , 1  , 0,
            -1 , 0  ,0,
            0 , 0  , 1;

    A << Matrix3d::Identity(),Matrix3d::Identity(),Matrix3d::Identity(),Matrix3d::Identity(),
        BP1x,BP2x,BP3x,BP4x;


    Ar_temp1 << B1R,MatrixXd::Zero(3,9),
        MatrixXd::Zero(3,3),B2R,MatrixXd::Zero(3,6),
        MatrixXd::Zero(3,6),B3R,MatrixXd::Zero(3,3),
        MatrixXd::Zero(3,9),B4R;


    Ar_temp2 = A*Ar_temp1;

    W1 << 0 , 0 , 1;
    W2 << 0 , 0 ,-1;
    W3 << 0 , 0 , 1;
    W4 << 0 , 0 , -1;
    W1 *= (BP1x*B1R);
    W2 *= (BP2x*B2R);
    W3 *= (BP3x*B3R);
    W4 *= (BP4x*B4R);

    Ar << Ar_temp2,W1,W2,W3,W4;

    ArT = (Ar.transpose())*((Ar * Ar.transpose()).inverse());

    u1.data.resize(3);
    u2.data.resize(3);
  
    Fd.data.resize(12);
    ros::Subscriber desire_thrust_total = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/desire_thrust_total",10,desire_thrust_total_cb); //corresponding u1 on matlab

    ros::Subscriber desire_moment_attitude = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/desire_total_moment",10,desire_moment_attitude_cb); // corresponding u2 on matlab

    ros::Subscriber desire_moment_angle = nh.subscribe<std_msgs::Float64>
        ("/gripper/desire_moment",10,desire_moment_angle_cb); // corresponding M on matlab

    ros::Publisher desire_thrust_each = nh.advertise<std_msgs::Float64MultiArray>
        ("/plarform/desire_thrust_each",10); 

    ROS_INFO("SUCCESS LAUNCH WRENCH_MAPPER!!!"); 

    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/platform/desire_thrust_total");

    ros::Rate rate(100);
    while(ros::ok())
    {  

        fd_gen();
        desire_thrust_each.publish(Fd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
