#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"

#include <Eigen/Dense>
#define lw  1; //units: meter

using namespace Eigen;

std_msgs::Float32MultiArray u1;
std_msgs::Float32MultiArray u2;
std_msgs::Float32 M;
std_msgs::Float64MultiArray Fd;


Matrix<float, 3, 1> BP1;
Matrix<float, 3, 1> BP2;
Matrix<float, 3, 1> BP3;
Matrix<float, 3, 1> BP4;

Matrix<float, 3, 3> BP1x;
Matrix<float, 3, 3> BP2x;
Matrix<float, 3, 3> BP3x;
Matrix<float, 3, 3> BP4x;

Matrix<float, 3, 3> B1R;
Matrix<float, 3, 3> B2R;
Matrix<float, 3, 3> B3R;
Matrix<float, 3, 3> B4R;


Matrix<float, 6, 12> A;
Matrix<float, 12,12> Ar_temp1;
Matrix<float, 6,12> Ar_temp2;
Matrix<float, 7,12> Ar;
Matrix<float, 12,7> ArT;
Matrix<float, 7, 1> fd_temp1;
Matrix<float, 12, 1> fd;
RowVector3f W1;
RowVector3f W2;
RowVector3f W3;
RowVector3f W4;

float Lw = lw;

void desire_thrust_total_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    u1 = *msg;

}

void desire_moment_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    u2 = *msg;
}
void desire_moment_angle_cb(const std_msgs::Float32::ConstPtr& msg)
{
    M = *msg;
    
}

void fd_gen()
{
    for(int r = 0; r< 3;r++)
    {
    fd_temp1(r,1) = u1.data[r];  
    }
    for(int r = 0; r< 3;r++)
    {
    fd_temp1(r+3,1) = u2.data[r];
    }
    fd_temp1(6,1) = M.data;
    fd = ArT* fd_temp1;
    for(int r = 0;r<12;r++)
    {
        Fd.data[r] = fd(r,1);

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

    BP1x << 0, -BP1[2],BP1[1],
            BP1[2],0,-BP1[0],
            -BP1[1],BP1[0],0;

    BP2x << 0,-BP2[2],BP2[1], 
            BP2[2],0,-BP2[0], 
            -BP2[1],BP2[0],0;

    BP3x << 0,-BP3[2],BP3[1],
            BP3[2],0,-BP3[0],
            -BP3[1],BP3[0],0;

    BP4x << 0,-BP4[2],BP4[1], 
            BP4[2],0,-BP4[0],
            -BP4[1],BP4[0],0;
    B1R = Matrix3f::Identity();
    B2R <<  0 ,-1  , 0,
            1 , 0  , 0,
            0 , 0  , 1;
    B3R << -1 , 0  , 0,
            0 ,-1  , 0,
            0 , 0  , 1;  
    B4R <<  0 , 1  , 0,
            -1 , 0  ,0,
            0 , 0  , 1;

    A << Matrix3f::Identity(),Matrix3f::Identity(),Matrix3f::Identity(),Matrix3f::Identity(),
        BP1x,BP2x,BP3x,BP4x;


    Ar_temp1 << B1R,MatrixXf::Zero(3,9),
        MatrixXf::Zero(3,3),B2R,MatrixXf::Zero(3,6),
        MatrixXf::Zero(3,6),B3R,MatrixXf::Zero(3,3),
        MatrixXf::Zero(3,9),B4R;


    Ar_temp2 = A*Ar_temp1;

    W1 << 0 , 0 , 1;
    W2 << 0 , 0 ,-1;
    W3 << 0 , 0 , 1;

    W1 *= (BP1x*B1R);
    W2 *= (BP2x*B2R);
    W3 *= (BP3x*B3R);
    W4 *= (BP4x*B4R);

    Ar << Ar_temp2,W1,W2,W3,W4;

    ArT = (Ar.transpose())*((Ar * Ar.transpose()).inverse());

    u1.data.resize(3);
    u2.data.resize(3);
  
    Fd.data.resize(12);
    ros::Subscriber desire_thrust_total = nh.subscribe<std_msgs::Float32MultiArray>
        ("/gripper/desire_thrust_total",10,desire_thrust_total_cb); //corresponding u1 on matlab

    ros::Subscriber desire_moment_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("/gripper/desire_moment_attitude",10,desire_moment_attitude_cb); // corresponding u2 on matlab

    ros::Subscriber desire_moment_angle = nh.subscribe<std_msgs::Float32>
        ("/gripper/desire_moment_angle",10,desire_moment_angle_cb); // corresponding M on matlab

    ros::Publisher desire_thrust_each = nh.advertise<std_msgs::Float64MultiArray>
        ("/gripper/desire_thrust_each",10); 

    ROS_INFO("SUCCESS LAUNCH WRENCH_MAPPER!!!"); 

    ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/gripper/desire_thrust_total");

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
