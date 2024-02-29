/*this file use to publish trajectory for gripper position,attitude*/
#include "getch.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "Eigen/Dense"

#define RADIUS 1

// using to switch to offboard(guide) & arming
mavros_msgs::SetMode sys_mode;
mavros_msgs::CommandBool arm_cmd;

geometry_msgs::PoseStamped system_pose;  //position
geometry_msgs::PoseStamped system_vel; //velocity

std_msgs::Float32MultiArray gripper;
std_msgs::Float32 phid;   //angle
std_msgs::Float32 phid_d; //angular vel
std_msgs::Float32 phid_dd; //angular acc
      
std_msgs::Int16 system_mode = -1;    //-1 = NON , 0 = GUIDE & ARMING , 2 = LAND  THIS IS A TRIGGER FOR NOD ,NOT ENTER!!!

enum {
    HOVERING_GRIPPER_STATIC,
    HOVERING_GRIPPER_SCISSORS,
    LAND,
}


/*function for ROS callback function*/

void mav_state_cb(const std_msgs::Int16::ConstPtr& msg)
{
    mav_state = *msg;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory");
    ros::NodeHandle nh;

    ros::Publisher system_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("system/trajectory_pose",10);
    ros::Publisher system_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("system/trajectory_vel",10);
    ros::Publisher system_gripper_pub = nh.advertise<std_msgs::Float32MultiArray>
        ("system/trajectory_gripper",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    ROS_INFO("(0):takeoff\n (1):hovering_gripper_stop\n (2):hovering_gripper_scissors\n (3):land");
    
    ros::Rate rate(50);
    while(ros::ok())
    {

        int c = getch();
        if(c != EOF){
        c_prev = c;
        switch (c)
        {
        case 0:
            {
                ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_STATIC!!!");
                ROS_INFO("PREPARING STE TO GUIDE MODE!!!");
                ROS_INFO("PREPARING STE TO ARMING!!!");


            }
            break;
        case 1:
            {
                ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_SCISSORS!!!");
                ROS_INFO("PREPARING HOVERING(STOP)!!!");
                hovering_gripper_stop();
            }
            break;
        case 2:
            {
                ROS_INFO("THE FLY TRAJECTORY IS: LAND!!!");
                ROS_INFO("PREPARING HOVERING(SCISSORS)!!!");
                ROS_INFO("PREPARING STE TO GUIDE MODE!!!");
                hovering_gripper_scissors();
            }
            break;
        }
        case 4:
            {
                ROS_INFO("PREPARING LANDING!!!");
                land();

            }
            break;
        }
        else if(c_prev !=-1)
        {



        }
        trajectory_generate();

        ros::spinOnce();
        rate.sleep();
    
    }
    return 0;
}