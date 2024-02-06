#include "ros/ros.h"
#include "constant.h" //define physical property
#include "std_msgs/Float32MultiArray.h"

#include <Eigen/Dense>

#define B 0
#define K 0
#define a 0
#define K1 0

float phi_error;
float phi_error_d;
float r;

std_msgs::Float32MultiArray phi;
std_msgs::Float32MultiArray phid;
std_msgs::Float32MultiArray M;
phi.data.resize(3);
phid.data.resize(3);
M.data.resize(3);
M.data[0] = 0;
M.data[1] = 0;
Gripper gripper;


void moment_calculator()



void phi_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    phi.data[0] = msg->data[0];
    phi.data[1] = msg->data[1];
    phi.data[2] = msg->data[2];

}

void phid_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    phid.data[0] = msg->data[0];
    phid.data[1] = msg->data[1];
    phid.data[2] = msg->data[2];

}


int main(int argc,char **argv)
{

    ros::init(argc,argv,"gripper_controller");
    ros::NodeHandle nh;

    ros::Subscriber phi_measure = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/phi_measure",10,phi_cb);

    ros::Subscriber phi_desire = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/phi_desire",10,phid_cb);

    ros::Publisher gripper_moment = nh.advertise<std_msgs::Float32MultiArray>
        ("gripper/desire_moment",10,);

    ros::Rate rate(100);

    while(ros::ok())
    {
        phi_error = phid.data[0] - phi.data[0];
        phi_error_d = phid.data[1] - phi.data[1];
        r = phi_error_d - a*phi_error;
        M.data[2] = gripper.ixyz[2]*(B*phi.data[1]+K*phi.data[0])+gripper.ixyz[2]*phid.data[2] \
        gripper.ixyz[2]*K1*r-gripper.ixyz[2]*(a^2)*phi_error;
        gripper_moment.publish(M);
        ros::spinOnce();
        rate.sleep;
    }

    return 0;
}


