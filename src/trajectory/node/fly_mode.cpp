/*this file use to publish trajectory for gripper position,attitude*/
#include "getch.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "mavros_msgs/State.h"
#include "Eigen/Dense"

#define RADIUS 1


mavros_msgs::State mav1_state, mav2_state ,mav3_state, mav4_state;

      
std_msgs::Int16 trajectory = -1;    //-1 = NON , 0 = GUIDE & ARMING , 2 = LAND  THIS IS A TRIGGER FOR NOD ,NOT ENTER!!!

std_msgs::Bool arm_signel = 0;  // 0 means disarm & land,1= arm & guide
std_msgs::Bool takeoff_signal = 0;

int c_prev = -1;

enum {
    HOVERING_GRIPPER_STATIC,
    HOVERING_GRIPPER_SCISSORS,
    LAND,
}
enum {
    ARM,
    Kill,
}


/*function for ROS callback function*/

void state_cb1(const mavros_msgs::State::ConstPtr& msg)
{
    mav1_state.mode = msg->mode;
    mav1_state.armed = msg->armed;
    
}

void state_cb2(const mavros_msgs::State::ConstPtr& msg)
{
    mav2_state.mode = msg->mode;
    mav2_state.armed = msg->armed;
    
}

void state_cb3(const mavros_msgs::State::ConstPtr& msg)
{
    mav3_state.mode = msg->mode;
    mav3_state.armed = msg->armed;
}

void state_cb4(const mavros_msgs::State::ConstPtr& msg)
{
    mav4_state.mode = msg->mode;
    mav4_state.armed = msg->armed;
   
}

int state() {
    if (mav1_state.mode == "GUIDED" && mav2_state.mode == "GUIDED" && 
        mav3_state.mode == "GUIDED" && mav4_state.mode == "GUIDED") {
        return 1;
    } else {
        return 0;
    }
}

int arm() {
    if (mav1_state.armed && mav2_state.armed  && 
        mav3_state.armed && mav4_state.armed) {
        return 1;
    } else {
        return 0;
    }
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"fly_mode");
    ros::NodeHandle nh;


//
    ros::Publisher system_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("system/trajectory_pose",10);
    ros::Publisher system_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("system/trajectory_vel",10);
    ros::Publisher system_gripper_pub = nh.advertise<std_msgs::Float32MultiArray>
        ("system/trajectory_gripper",10);

    ros::Subscriber MAV1 = nh.subscribe<mavros_msgs::State>
        ("/MAV1/mavros/state",10,state_cb1);
    ros::Subscriber MAV1 = nh.subscribe<mavros_msgs::State>
        ("/MAV2/mavros/state",10,state_cb2);
    ros::Subscriber MAV1 = nh.subscribe<mavros_msgs::State>
        ("/MAV3/mavros/state",10,state_cb3);
    ros::Subscriber MAV1 = nh.subscribe<mavros_msgs::State>
        ("/MAV4/mavros/state",10,state_cb4);
    ros::Publisher system_trajectory = nh.advertise<std_msgs::Int16>
        ("system/trajectory");

    ros::Publisher MAV_arm = nh.advertise<std_msgs::Bool>
        ("MAV/arm");

    ros::Publisher MAV_takeoff = nh.advertise<std_msgs::Bool>
        ("MAV/takeoff");


    ROS_INFO("(0):takeoff\n (1):hovering_gripper_stop\n (2):hovering_gripper_scissors\n (3):land");
    
    ros::Rate rate(50);
    while(ros::ok()c_prev)
    {

        int c = getch();
        if(c != EOF){
            switch (c)
            {
            case 0:
                {
                    
                        trajectory.data = HOVERING_GRIPPER_STATIC;
                        arm_signel.data = ARM;
                        ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_STATIC!!!");
                        system_trajectory.publish(trajectory.data);
                        MAV_arm.publish(arm_signel.data);
                        ROS_INFO("PREPARING STE TO GUIDE MODE!!!");
                        ROS_INFO("PREPARING STE TO ARMING!!!");
                    
                        while(ros::ok() && state() == 1 && arm() == 1)
                        {
                            ROS_WARN("WAIT_ALL_MAV_ARE_READY");
                            ros::spinOnce();
                            rate.sleep();
                        }
                        ROS_WARN("READY_TAKEOFF!!!");
                        takeofMAV_takeofff_signal.data= 1;
                        MAV_takeoff.publish(takeofMAV_takeofff_signal.data);
                    
                
                }
                break;
            case 1:
                {
                    if(c != c_prev){
                        trajectory.data = HOVERING_GRIPPER_SCISSORS;
                        arm_signel.data = ARM;
                        ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_SCISSORS!!!");
                        system_trajectory.publish(trajectory.data);
                        MAV_arm.publish(arm_signel.data);
                        ROS_INFO("PREPARING STE TO GUIDE MODE!!!!!!");
                        ROS_INFO("PREPARING STE TO ARMING!!!");
                        }
                       while(ros::ok() && state() == 1 && arm() == 1)
                       {
                           ROS_WARN("WAIT_ALL_MAV_ARE_READY");
                           ros::spinOnce();
                           rate.sleep();
                       }
          
                        ROS_WARN("READY_TAKEOFF!!!");
                        takeofMAV_takeofff_signal.data= 1;
                        MAV_takeoff.publish(takeofMAV_takeofff_signal.data);


                }
                break;
            case 2:
                {
                    trajectory.data = LAND;
                    arm_signel.data = Kill;
                    ROS_INFO("THE FLY TRAJECTORY IS: LAND!!!");
                    system_trajectory.publish(trajectory.data);
                    MAV_arm.publish(arm_signel.data);
                    ROS_INFO("PREPARING SET TO LAND MODE!!!");
                    ROS_INFO("PREPARING STE TO DISARM !!!");
                }
                break;
            }
            }
    
        ros::spinOnce();
        rate.sleep();
    
        }
    return 0;
}