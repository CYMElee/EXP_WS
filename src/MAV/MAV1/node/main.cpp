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


enum MAV_mod{
    IDLE,
    TAKEOFF,
    LAND,
    SET_HOME,
};

int UAV_ID;
/*use to publish*/
geometry_msgs::PoseStamped pose;

/*platform pose*/
geometry_msgs::PoseStamped platform_pose;
/*MAV pose*/
geometry_msgs::PoseStamped mav_pose;

mavros_msgs::State current_state;
 
std_msgs::Float64MultiArray T_cmd;
std_msgs::Float64MultiArray Eul_cmd;

std_msgs::Int16 Change_Mode_Trigger ;

mavros_msgs::AttitudeTarget T;
mavros_msgs::AttitudeTarget T_PREARM;

void initialize(void);

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void mode_cb(const std_msgs::Int16::ConstPtr& msg);
void platform_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void mav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void T_sub_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
void T_cmd_calculate(void);




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

    //*******************************//
    //        use after takeoff       //
    //******************************//

    /*subscribe the Platform attitude*/
    ros::Subscriber platform_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/platform/mavros/local_position/pose",12,platform_pose_cb);
    /*subscribe the MAV attitude, get from optitrack*/
    std::string sub_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/local_position/pose");
    ros::Subscriber MAV_pose = nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, mav_pose_cb);
    /*****Subscribe the Gimbal_angle and Thrust command!!!******/
    ros::Subscriber T_sub = nh.subscribe<std_msgs::Float64MultiArray>("cmd",12,T_sub_cb);

    /****Publish the Attitude and Thrust command  !!!****/
    ros::Publisher T_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("mavros/setpoint_raw/attitude", 10);
    ros::Subscriber Takeoff_Signal = nh.subscribe<std_msgs::Int16>("/ground_station/set_mode", 10, mode_cb);

    /*** using debug ***/
    ros::Publisher T_pub_debug = nh.advertise<std_msgs::Float32MultiArray>
        ("mavros/setpoint_raw/attitude_euler", 10);

    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

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

    rate = ros::Rate(100);
    while(ros::ok() && Change_Mode_Trigger.data !=MAV_mod::TAKEOFF){

        ROS_INFO("READY_TAKEOFF!!");
        //if( current_state.mode != "OFFBOARD" ){
          //  set_mode_client.call(offb_set_mode);
       // }
       // if(!current_state.armed){
       //     arming_client.call(arm_cmd);
       // }
    T_pub.publish(T_PREARM);
	ros::spinOnce();
	rate.sleep();
    }
    rate = ros::Rate(100);
    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("cmd");
    while(ros::ok()){

        T_cmd_calculate();
        T_pub.publish(T);
        T_pub_debug.publish(Eul_cmd);

        ros::spinOnce();
        rate.sleep();
    }

   
    return 0;
}


void T_sub_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    T_cmd = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void mode_cb(const std_msgs::Int16::ConstPtr& msg){
	Change_Mode_Trigger = *msg;
}

void platform_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    platform_pose = *msg;
}
void mav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mav_pose = *msg;
}

void initialize(void){
    //init the attitude and thrust value
    T.type_mask = 7 ;
    T_cmd.data.resize(3);
    T_PREARM.type_mask = 7 ;
    T_PREARM.orientation.w = 1;
    T_PREARM.orientation.x = 0;
    T_PREARM.orientation.y = 0;
    T_PREARM.orientation.z = 0;
    T_PREARM.thrust = 0.3;  
    Eul_cmd.data.resize(3);
    
    

}


void T_cmd_calculate(void){

    /*in here we will change the mav pose from quater to euler angle*/
    Eigen::Quaterniond quaternion_mav(mav_pose.pose.orientation.w,mav_pose.pose.orientation.x,mav_pose.pose.orientation.y,mav_pose.pose.orientation.z );
    Eigen::Matrix3d rotationMatrix_mav = quaternion_mav.toRotationMatrix();
    Eigen::Vector3d eulerAngles_mav = rotationMatrix_mav.eulerAngles(2, 1, 0);
    double alpha = T_cmd.data[1];
    double  beta = T_cmd.data[2];
    double  thrust = T_cmd.data[0]; 
    double  z = eulerAngles_mav(0);

     /*get the MAV desire Attitude(from mav fram to platform fram)*/
    Eigen::Quaterniond MAV_pose_cmd;
    MAV_pose_cmd = Eigen::AngleAxisd(z,Eigen::Vector3d::UnitZ()) * \
    Eigen::AngleAxisd(beta,Eigen::Vector3d::UnitY()) * \
    Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotationMatrix_mav_des = MAV_pose_cmd.toRotationMatrix();




    /**Build the Rotation matrix(platform body to inertial)**/
    Eigen::Quaterniond quaternion_platform(platform_pose.pose.orientation.w,platform_pose.pose.orientation.x,platform_pose.pose.orientation.y,platform_pose.pose.orientation.z);
    Eigen::Matrix3d rotationMatrix_platform = quaternion_platform.toRotationMatrix();

    /**get the desire mav attitude (body frame to inertial frame)**/
    Eigen::Matrix3d rotationMatrix_mav_des_b2i = rotationMatrix_platform*rotationMatrix_mav_des;
    
    Eigen::Vector3d eulerAngles_mav_des = rotationMatrix_mav_des_b2i.eulerAngles(2, 1, 0);
  
  
    Eigen::Quaterniond mav_pose_desire;
    mav_pose_desire = Eigen::AngleAxisd(eulerAngles_mav_des(0),Eigen::Vector3d::UnitZ()) * \
    Eigen::AngleAxisd(eulerAngles_mav_des(1),Eigen::Vector3d::UnitY()) * \
    Eigen::AngleAxisd(eulerAngles_mav_des(2),Eigen::Vector3d::UnitX());

    Eul_cmd.data[0] = eulerAngles_mav_des(0);
    Eul_cmd.data[1] = eulerAngles_mav_des(1);
    Eul_cmd.data[2] = eulerAngles_mav_des(2);
    

    T.orientation.w =mav_pose_desire.w();
    T.orientation.x =mav_pose_desire.x();
    T.orientation.y =mav_pose_desire.y();
    T.orientation.z =mav_pose_desire.z();
    T.thrust = thrust;

}


