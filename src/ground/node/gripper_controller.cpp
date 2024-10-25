#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"



#define Iz 6e-5


#define B 0
#define K 0
#define a 1
#define K1 800

float phi_error = 0;
float phi_error_d = 0;
float r = 0.0001;


std_msgs::Float64MultiArray phi;
std_msgs::Float64MultiArray phid;
std_msgs::Float64 M;






//void phi_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
//{
 //   phi = *msg;
//}

void initialize(void);

void phid_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    phid= *msg;
}


int main(int argc,char **argv)
{

    ros::init(argc,argv,"gripper_controller");
    ros::NodeHandle nh;
    //initalize...
    initialize();
    ROS_INFO("SUCCESS LAUNCH GRIPPER CONTROLLER");

    

  //  ros::Subscriber phi_measure = nh.subscribe<std_msgs::Float32MultiArray>
 //       ("/gripper/phi_measure",10,phi_cb);

    ros::Subscriber phi_desire = nh.subscribe<std_msgs::Float64MultiArray>
        ("/gripper/phi_desire",10,phid_cb);

    ros::Publisher gripper_moment = nh.advertise<std_msgs::Float64>
        ("/gripper/desire_moment",10);
   // ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/gripper/phi_desire");
    ros::Rate rate(100);
    while(ros::ok())
    {
        
        phi_error = phid.data[0] - phi.data[0];
        phi_error_d = phid.data[1] - phi.data[1];
        r = phi_error_d + a*phi_error;
        M.data = Iz*(B*phi.data[1]+K*phi.data[0])+Iz*phid.data[2]+\
        (Iz*K1*r)-(Iz*(a*a)*phi_error);
        M.data = (M.data/2);
        gripper_moment.publish(M);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



void initialize(void){
    phi.data.resize(3);
    phid.data.resize(3);
    phi.data[0] = 1.57;
    phi.data[1] = 0;
    phi.data[2] = 0;
    phid.data[0] = 1.57;
    phid.data[1] = 0;
    phid.data[2] = 0;
    M.data = 0;
}


