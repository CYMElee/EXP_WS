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
#include "std_msgs/Float32MultiArray.h"
#include "cmath"
#include "Eigen/Dense"


using namespace Eigen;



enum MAV_mod{
    IDLE,
    TAKEOFF,
    LAND,
};

int UAV_ID;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped mav_receive_pose;
mavros_msgs::State current_state;
std_msgs::Int16 Change_Mode_Trigger ;
mavros_msgs::AttitudeTarget T;
std_msgs::Float64MultiArray Attitude_Thrust_receive;

Matrix<float, 3,3> PLA_R;
Matrix<float, 3,3> MAV_R;
Matrix<float, 3,3> PLA2MAV_R;
Matrix<float, 3,3> DES_GIMBAL_R;
Matrix<float,3,3> DES_MAV_R;

Vector3f MAV_EUL;
Vector2f GIMBAL_ANG;

Quaternionf platform_pose;
Quaternionf mav_pose;
Quaternionf mav_pose_desire;



void state_cb(const mavros_msgs::State::ConstPtr& msg);
void Takeoff_Signal_cb(const std_msgs::Int16::ConstPtr& msg);
void T_sub_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);

void initialize(void);

void Trans_Gimbal2Attitude(void);
void platform_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);

void mav_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);


int main(int argv,char** argc)
{
    ros::init(argv,argc,"main");
    ros::NodeHandle nh;
    ros::param::get("UAV_ID", UAV_ID);
    std::string sub_topic = std::string("/vrpn_client_node/MAV") + std::to_string(UAV_ID) + std::string("/pose");
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

    //*******************************//
    //        use after takeoff       //
    //******************************//


    /*subscribe the Platform attitude*/
    ros::Subscriber PLA_Atti_sub = nh.subscribe<std_msgs::Float32MultiArray>("/platform/measure_attitude",12,platform_attitude_cb);
    /*subscribe the MAV attitude, get from optitrack*/
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, mav_attitude_cb);
    /*****Subscribe the Gimbal_angle and Thrust command!!!******/
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
        Trans_Gimbal2Attitude();

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
    T.thrust = msg->data[0]; // thrust
    GIMBAL_ANG(0,1) = msg->data[1]; //Alpha
    GIMBAL_ANG(1,1) = msg->data[2]; //Beta    
}

void platform_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){

    platform_pose = AngleAxisf(msg->data[0],\
    Vector3f(msg->data[1], msg->data[2], msg->data[3]));
    PLA_R = platform_pose.toRotationMatrix();
}

void mav_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    mav_pose = AngleAxisf(msg->pose.orientation.w,\
    Vector3f(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z));
    MAV_R = mav_pose.toRotationMatrix();
    MAV_EUL = MAV_R.eulerAngles(2, 1, 0);
}



void initialize(void){

    //init the attitude and thrust value
    T.orientation.w = 1;
    T.orientation.x = 0;
    T.orientation.y = 0;
    T.orientation.z = 0;
    T.thrust = 0.1;
    T.type_mask = 7 ;
    
}



void Trans_Gimbal2Attitude(void){
    DES_GIMBAL_R  = AngleAxisf(MAV_EUL(0), Vector3f::UnitZ())*\
                    AngleAxisf(GIMBAL_ANG(0,1), Vector3f::UnitY())*\
                    AngleAxisf(GIMBAL_ANG(1,1), Vector3f::UnitX());  

    DES_MAV_R = PLA_R*DES_GIMBAL_R;

    mav_pose_desire = Quaternionf(DES_MAV_R);
    T.orientation.w = mav_pose_desire.w();
    T.orientation.x = mav_pose_desire.x();
    T.orientation.y = mav_pose_desire.y();
    T.orientation.z = mav_pose_desire.z();

}