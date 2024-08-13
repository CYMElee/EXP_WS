#include "ros/ros.h"
#include <string>
#include "std_msgs/Int16.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include "mavros_msgs/AttitudeTarget.h"
#include "std_msgs/Float64MultiArray.h"
#include "cmath"
#include "Eigen/Dense"


enum MAV_mod{
    IDLE,
    TAKEOFF,
    LAND,
};

int UAV_ID;
geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
std_msgs::Int16 Change_Mode_Trigger ;

mavros_msgs::AttitudeTarget T;

std_msgs::Float64MultiArray Attitude_Thrust_receive;





void state_cb(const mavros_msgs::State::ConstPtr& msg);
void Takeoff_Signal_cb(const std_msgs::Int16::ConstPtr& msg);
void T_sub_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);

void initialize(void);


int main(int argv,char** argc)
{
    ros::init(argv,argc,"main");
    ros::NodeHandle nh;
    ros::param::get("UAV_ID", UAV_ID);
    // initialize init param and system state//
    initialize();//set the init Attitude and Thrust
    Change_Mode_Trigger.data=MAV_mod::IDLE;
    //*******************************//
    //        use for takeoff       //
    //******************************//



    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");


    /*****Subscribe the Attitude and Thrust command!!!******/
    ros::Subscriber T_sub = nh.subscribe<std_msgs::Float64MultiArray>("cmd",12,T_sub_cb);

    /****Publish the Attitude and Thrust command  !!!****/
    ros::Publisher T_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("mavros/setpoint_raw/attitude", 10);
    ros::Subscriber Takeoff_Signal = nh.subscribe<std_msgs::Int16>("/ground_station/set_mode", 10, Takeoff_Signal_cb);
    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::param::get("UAV_ID", UAV_ID);

    ROS_INFO("Wait for setting origin and home position...");
    std::string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic);
    ROS_INFO("Message received or timeout reached. Continuing execution.");
 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    //send a few setpoints before starting

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;

    rate = ros::Rate(100);
    while(ros::ok() && Change_Mode_Trigger.data !=MAV_mod::TAKEOFF){

        ROS_INFO("READY_TAKEOFF!!");

        T_pub.publish(T); //idle

	    ros::spinOnce();
	    rate.sleep();


    }
    rate = ros::Rate(100);
    
    while(ros::ok()){
        /*********************************/
        /**  if receive the LAND topic  */
        /*******************************/
        if(Change_Mode_Trigger.data ==MAV_mod::LAND){
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }

        T_pub.publish(T);

        ros::spinOnce();
        rate.sleep();
    }

   
    return 0;
}




void state_cb(const mavros_msgs::State::ConstPtr& msg){

	current_state = *msg;
}

void Takeoff_Signal_cb(const std_msgs::Int16::ConstPtr& msg){

	Change_Mode_Trigger = *msg;
}
void T_sub_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    T.orientation.w = msg->data[0];
    T.orientation.x = msg->data[1];
    T.orientation.y = msg->data[2];
    T.orientation.z = msg->data[3];
    T.thrust = msg->data[4];

}

void initialize(void){

    //init the attitude and thrust value
    T.orientation.w = 1;
    T.orientation.x = 0;
    T.orientation.y = 0;
    T.orientation.z = 0;
    T.thrust = 0.1;
    T.type_mask = T.IGNORE_PITCH_RATE | \
    T.IGNORE_ROLL_RATE |T.IGNORE_YAW_RATE ;
    
}

