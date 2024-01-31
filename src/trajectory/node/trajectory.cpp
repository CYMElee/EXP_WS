/*this file use to publish trajectory for gripper position,attitude*/
#include "getch.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"


#include "Eigen/Dense"

geometry_msgs::PoseStamped gripper_pose;
geometry_msgs::TwistStamped gripper_vel;


int gripper_mode;
int arming = 0; //0 means that disarm
int gripper_cd = 0;
enum {

    TAKEOFF,
    HOVERING_GRIPPER_SCISSORS,
    HOVERING_GRIPPER_STATIC,
    CIRCLE,
    LAND,
}Gripper_Mode;
void take_off()
{
    if(gripper_mode == TAKEOFF && gripper_cd == 1)
    {
        ROS_INFO("READY TAKEOFF!!!")
    }
    else
    {
        gripper_mode = TAKEOFF;
        ROS_INFO("START ARMING!!!");
    }

}

/*function for gripper mode select*/
void hovering_gripper_stop()
{




}

void hovering_gripper_scissors()
{



}

void land()
{



}

/*function for ROS callback function*/




int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    
    ros::Publisher gripper_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode")


    ROS_INFO("(1):takeoff\n (2):hovering_gripper_stop\n (3):hovering_gripper_scissors\n (4):land");
    
    while(ros::ok())
    {
        int c = getch();

        if(c != EOF){
        switch (c)
        {
        case 0:
            {
                ROS_INFO("PREPARING ARMING!!!");
                take_off();


            }
            break;
        case 1:
            {
                ROS_INFO("PREPARING HOVERING(STOP)!!!");
                hovering_gripper_stop();
            }
            break;
        case 2:
            {
                ROS_INFO("PREPARING HOVERING(SCISSORS)!!!");
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

    
    }
    return 0;
}