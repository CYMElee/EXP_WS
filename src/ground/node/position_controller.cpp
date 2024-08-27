#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"

#define G 9.81 //greavity
#define  M_main 0.4 //kg
#define  M_qc  0.4 //kg


using namespace Eigen;

std_msgs::Float32MultiArray t; //desire_thrust

Matrix<float, 3, 1> p;
Matrix<float, 3, 1> pd;
Matrix<float, 3, 1> p_dot;
Matrix<float, 3, 1> pd_dot;
Matrix<float, 3, 1> u1;
Matrix<float, 3, 1> fr;

Matrix<float, 3, 3> Kp;
Matrix<float, 3, 3> Kv;
Matrix<float, 3, 3> Ki;
Matrix<float, 3, 3> R;

Matrix<float, 3, 1> ep;
Matrix<float, 3, 1> ev;

Matrix<float, 3, 1> z; //0,0,1

Quaternionf quaternion;



void desire_position_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    pd(0) = msg->data[0]; //x
    pd(1)= msg->data[1];  //y
    pd(2) = msg->data[2]; //z

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
   
  quaternion = AngleAxisf(msg->data[0],Vector3f(msg->data[1], msg->data[2], msg->data[3]));

   R = quaternion.toRotationMatrix();

    
}


void total_thrust()
{
    ep(0) = p(0) - pd(0);
    ep(1) = p(1) - pd(1);
    ep(2) = p(2) - pd(2);
    
    ev(0) = p_dot(0) - pd_dot(0);
    ev(1) = p_dot(1) - pd_dot(1);
    ev(2) = p_dot(2) - pd_dot(2);

    fr = ((M_qc+M_main/2)*4+M_main)*(G*z-Kp*ep-Kv*ev);
    u1 = R.transpose()*fr; // transform the desire thrust from inertial frame to body fixed frame
    
    t.data[0] = u1(0);
    t.data[1] = u1(1);
    t.data[2] = u1(2);
    //ROS_INFO("THE THRUST:%f",t.data[0]);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"position_controller");
    ros::NodeHandle nh;
    t.data.resize(3);
    ROS_INFO("SUCCESS LAUNCH POSITION CONTROLLER"); 

    //control gain
Kp << 10, 0 , 0 ,
      0  , 10 ,0 ,
      0  , 0,  5;

Kv << 0 , 0  ,0 ,
      0  , 0  ,0,
      0  , 0  ,0;

Ki << 1, 0 , 0 ,
      0  , 1,0,
      0  , 0,  1;
z<< 0,0,1;

    ros::Subscriber desire_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("/platform/desire_position",10,desire_position_cb); 

    ros::Subscriber desire_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("/platform/desire_velocity",10,desire_velocity_cb);

    ros::Subscriber measure_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("/platform/measure_position",10,measure_position_cb); 

    ros::Subscriber measure_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10,measure_velocity_cb); 

    ros::Subscriber measure_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("/platform/measure_attitude",10,measure_attitude_cb); 

    ros::Publisher desire_thrust_total = nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/desire_thrust_total",10);

    ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/platform/desire_position");
    
    ROS_INFO("RECEIVE POSITION");
    ros::Rate rate(100);
    while(ros::ok())
    {
        
        
        total_thrust();
        desire_thrust_total.publish(t);

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}