/*this file use to publish trajectory for gripper position,attitude*/
#include "ros/ros.h"
#include "ex_ground/getch.h"
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

      
std_msgs::Int16 trajectory;   //-1 = NON , 0 = GUIDE & ARMING , 2 = LAND  THIS IS A TRIGGER FOR NOD ,NOT ENTER!!!

std_msgs::Bool arm_signel; // 0 means disarm & land,1= arm & guide
std_msgs::Bool kill;

std_msgs::Bool takeoff_signal;

int c_prev = -1;

enum {
    HOVERING_GRIPPER_STATIC,
    HOVERING_GRIPPER_SCISSORS,
    LAND,
}TRAJECTORY;
enum {
    ARM,
    Kill,
}MAV_mod;


/*function for ROS callback function*/

void state_cb1(const mavros_msgs::State::ConstPtr& msg)
{
    mav1_state = *msg;
    
}

void state_cb2(const mavros_msgs::State::ConstPtr& msg)
{
    mav2_state = *msg;
    
    
}

void state_cb3(const mavros_msgs::State::ConstPtr& msg)
{
    mav3_state = *msg;
   
}

void state_cb4(const mavros_msgs::State::ConstPtr& msg)
{
    mav4_state = *msg;
    
   
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
    int c_prev = EOF;
    trajectory.data = -1;
    arm_signel.data = 0;
    takeoff_signal.data  = 0;
    kill.data = 1;

    ros::Subscriber MAV1 = nh.subscribe<mavros_msgs::State>
        ("/uav0/mavros/state",10,state_cb1);
    ros::Subscriber MAV2 = nh.subscribe<mavros_msgs::State>
        ("/uav1/mavros/state",10,state_cb2);
    ros::Subscriber MAV3 = nh.subscribe<mavros_msgs::State>
        ("/uav2/mavros/state",10,state_cb3);
    ros::Subscriber MAV4 = nh.subscribe<mavros_msgs::State>
        ("/uav3/mavros/state",10,state_cb4);
    ros::Publisher system_trajectory = nh.advertise<std_msgs::Int16>("/system/trajectory",10);
    ros::Publisher system_kill = nh.advertise<std_msgs::Bool>("/system/kill",10);

    ros::Publisher MAV_takeoff = nh.advertise<std_msgs::Bool>("/MAV/takeoff",10);


    ROS_INFO("\n(0):hovering_gripper_stop\n (1):hovering_gripper_scissors\n (2):land");
    
    ros::Rate rate(20);
    while(ros::ok())
    {
       
        int c = getch();
        if(c == EOF)
        {
            c = c_prev;
        }
        if(c != EOF){
            switch (c)
            {
            case 'A':
                {
                        trajectory.data = HOVERING_GRIPPER_STATIC;
                        ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_STATIC!!!");
                        system_trajectory.publish(trajectory);
                        ROS_INFO("PREPARING STE TO GUIDE MODE!!!");
                        ROS_INFO("PREPARING STE TO ARMING!!!");
                    
                        while(ros::ok() && arm() == 0)
                        {
                            ROS_WARN("WAIT_ALL_MAV_ARE_READY");
                            ros::spinOnce();
                            rate.sleep();
                        }
                        ROS_WARN("READY_TAKEOFF!!!");
                        takeoff_signal.data = 1;
                        MAV_takeoff.publish(takeoff_signal);
                    
                
                }
                break;
            case 'B':
                {
                    if(c != c_prev){
                        trajectory.data = HOVERING_GRIPPER_SCISSORS;
                        ROS_INFO("THE FLY TRAJECTORY IS: HOVERING_GRIPPER_SCISSORS!!!");
                        system_trajectory.publish(trajectory);
                        ROS_INFO("PREPARING STE TO GUIDE MODE!!!!!!");
                        ROS_INFO("PREPARING STE TO ARMING!!!");
                        }
                       while(ros::ok() && arm() == 0)
                       {
                           ROS_WARN("WAIT_ALL_MAV_ARE_READY");
                           ros::spinOnce();
                           rate.sleep();
                       }
          
                        ROS_WARN("READY_TAKEOFF!!!");
                        takeoff_signal.data = 1;
                        MAV_takeoff.publish(takeoff_signal);


                }
                break;
            case 'C':
                {
                    trajectory.data = LAND;
                    ros::Time time_out = ros::Time::now();
                    while(ros::ok() && ros::Time::now()-time_out<ros::Duration(30))
                    {
                    ROS_INFO("THE FLY TRAJECTORY IS: LAND!!!");
                    system_trajectory.publish(trajectory);
                    ROS_INFO("PREPARING SET TO LAND MODE!!!");
                    ROS_INFO("PREPARING STE TO DISARM !!!");
                    ros::spinOnce();
                    rate.sleep();
                    }   
                }
                break;
            }
            }
        c_prev = c;
        ros::spinOnce();
        rate.sleep();
    
        }
    return 0;
}