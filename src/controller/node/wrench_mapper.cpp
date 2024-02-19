#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "constant.h"
#include <Eigen/Dense>

using namespace Eigen;

std_msgs::Float32MultiArray desire_thrust;
std_msgs::Float32MultiArray desire_moment_attitude;
std_msgs::Float32MultiArray desire_moment_angle;
std_msgs::Float32MultiArray fd;
desire_thrust.data.resize(3);
desire_moment_attitude.data.resize(3);
desire_moment_angle.data.resize(3);
fd.data.resize(12);


Gripper gripper;
float Lw = gripper.Lw;

Vector3f BP1;
Vector3f BP2;
Vector3f BP3;
Vector3f BP4;

Matrix<float, 3, 3> BP1x;
Matrix<float, 3, 3> BP2x;
Matrix<float, 3, 3> BP3x;
Matrix<float, 3, 3> BP4x;

Matrix<int, 3, 3> B1R;
Matrix<int, 3, 3> B2R;
Matrix<int, 3, 3> B3R;
Matrix<int, 3, 3> B4R;


Matrix<float, 6, 12> A;
Matrix<float, 12,12> Ar_temp1;
Matrix<float, 6,12> Ar_temp2;
Matrix<float, 7,12> Ar;
Matrix<float, 12,7> ArT;
Matrix<float, 7, 1> fd_temp1;
Matrix<float, 12, 1> fd;
RowVector3d W1;
RowVector3d W2;
RowVector3d W3;
RowVector3d W4;

BP1 << Lw, 0, 0 ;
BP2 << 0, Lw, 0 ;
BP3 << -Lw, 0, 0 ;
BP4 << 0, -Lw, 0 ;

BP1x << 0, -BP[2],BP[1],
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

B1R = Matrix3d::Identity();
B2R <<  0 ,-1  , 0
        1 , 0  , 0
        0 , 0  , 1;
B3R << -1 , 0  , 0
        0 ,-1  , 0
        0 , 0  , 1;  
B4R <<  0 , 1  , 0
       -1 , 0  , 0
        0 , 0  , 1; 


A << Matrix3d::Identity(),Matrix3d::Identity(),Matrix3d::Identity(),
     BP3x,BP2x,BP3x,BP4x;


Ar_temp1 << B1R,MatrixXd::Zero(3,9),
      MatrixXd::Zero(3,3),B2R,MatrixXd::Zero(3,6),
      MatrixXd::Zero(3,6),B3R,MatrixXd::Zero(3,3),
      MatrixXd::Zero(3,9),B4R;


Ar_temp2 = A*Ar_temp1;

W1 << 0 , 0 , 1;
W2 << 0 , 0 ,-1;
W3 << 0 , 0 , 1;

W1 *= (BP1x*B1R);
W2 *= (BP2x*B2R);
W3 *= (BP3x*B3R);
W4 *= (BP4x*B4R);MultiArray

Ar << Ar_temp2,W1MultiArrayW2,W3;

ArT = (Ar.transpoMultiArraye())*((Ar * Ar.transpose()).inverse());

void desire_thrus_total_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
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
void desire_moment_angle_cb(const std_msgs::Float32::ConstPtr& msg)
{
    desire_moment_angle = *msg;
    
}

void fd_gen()
{
    for(rt = 1; r< 4;r++)
    {
    fd_temp1(rt,1) = desire_thrust.data[rt-1];  
    }
    for(rm_a = 1; r< 4;r++)
    {
    fd_temp1(rt+3,1) = desire_thrust.data[rm_a-1];
    }
    fd_temp1(7,1) = desire_moment_angle;
    fd = ArT* fd_temp1;

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"wrench_mapper");
    ros::NodeHandle nh;

    ros::Subscriber desire_thrust_total = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_thrust_total",10,desire_thrust_total_cb); //corresponding u1 on matlab

    ros::Subscriber desire_moment_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_moment_attitude",10,desire_moment_attitude_cb); // corresponding u2 on matlab

    ros::Subscriber desire_moment_angle = nh.subscribe<std_msgs::Float32>
        ("gripper/desire_moment_angle",10,desire_moment_angle_cb); // corresponding M on matlab

    ros::Publisher desire_thrust_each = nh.advertise<std_msgs::Float32MultiArray>
        ("gripper/desire_thrust_each",10); 

    ros::Rate rate(100);
    while(ros::ok())
    {  

        fd_gen();
        desire_thrust_each.publish(fd);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
