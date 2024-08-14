#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include "geometry_msgs/PoseStamped.h"

#define D2R 0.0174
#define dt 0.01


std_msgs::Float32MultiArray pd;
std_msgs::Float32MultiArray pd_d;
std_msgs::Float32MultiArray Rr;
std_msgs::Float32MultiArray agvr;
std_msgs::Float32MultiArray phid;
std_msgs::Int16 Mode;
geometry_msgs::PoseStamped pose;

enum MAV_mod{
    IDLE,
    TAKEOFF,
    LAND,
};


void initialize(void);

void hovering(void);

void land(void);

void mode_cb(const std_msgs::Int16::ConstPtr& msg){
    Mode = *msg;

}

//void pose_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
//    pose = *msg;
//}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory_generator");
    ros::NodeHandle nh;
    /*initial some variable*/
    initialize();

    /*get the system's pose*/
   // ros::Subscriber GET_POSE = nh.subscribe<std_msgs::Float32MultiArray>
      //  ("/platform/measure_position",10,pose_cb);

    /*get the system's fly mode*/
    ros::Subscriber GET_MODE = nh.subscribe<std_msgs::Int16>
        ("/ground_station/set_mode",10,mode_cb);
    /*publish the trajectory*/
    ros::Publisher desire_position = nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/desire_position",10);
    ros::Publisher desire_velocity = nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/desire_velocity",10);
    ros::Publisher desire_attitude = nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/desire_attitude",10);
    ros::Publisher desire_omega = nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/desire_omega",10);
    ros::Publisher desire_phi = nh.advertise<std_msgs::Float32MultiArray>
        ("/gripper/phi_desire",10);

    ros::Rate rate(100);


    while(ros::ok())
    {
        if(Mode.data = MAV_mod::IDLE){
           
        }

        if(Mode.data = MAV_mod::TAKEOFF){
          hovering();  

        }


        if(Mode.data = MAV_mod::LAND){
            land();
        }


    }

    desire_position.publish(pd);
    desire_velocity.publish(pd_d);
    desire_attitude.publish(Rr);
    desire_omega.publish(agvr);
    desire_phi.publish(agvr);

    ros::spinOnce();
    rate.sleep();

    return 0;
}


void initialize(void){
    Mode.data = MAV_mod::IDLE;
    pd.data.resize(3);
    pd_d.data.resize(3);
    Rr.data.resize(3);
    agvr.data.resize(3);
    phid.data.resize(3);

}


void hovering(void){

/*in hovering mode, we hope the system(platform) can hovering on z=0.5m */

    /*platform position*/
    pd.data[0] = 1;
    pd.data[1] = 1;
    pd.data[2] = 0.5;

    /*platform velocity*/
    pd_d.data[0] = 0;
    pd_d.data[1] = 0;
    pd_d.data[2] = 0;

    /*platform attitude*/
    Rr.data[0] = 1;
    Rr.data[1] = 0;
    Rr.data[2] = 0;
    Rr.data[3] = 0;

    /*platform rate change*/

    agvr.data[0] = 0;
    agvr.data[1] = 0;
    agvr.data[2] = 0;

    /*platform gripper angle*/
    phid.data[0] = 1.57;
    phid.data[1] = 0;
    phid.data[2] = 0;



}

void land(void){
    pd.data[0] = 1;
    pd.data[1] = 1;
    pd.data[2] = 0.2;

    /*platform velocity*/
    pd_d.data[0] = 0;
    pd_d.data[1] = 0;
    pd_d.data[2] = 0;

    /*platform attitude*/
    Rr.data[0] = 1;
    Rr.data[1] = 0;
    Rr.data[2] = 0;
    Rr.data[3] = 0;

    /*platform rate change*/

    agvr.data[0] = 0;
    agvr.data[1] = 0;
    agvr.data[2] = 0;

    /*platform gripper angle*/
    phid.data[0] = 1.57;
    phid.data[1] = 0;
    phid.data[2] = 0;

}
