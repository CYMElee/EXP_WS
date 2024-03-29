#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "math.h"

#define D2R 0.0174
#define dt 0.001

/*[0] is 0 derivate [1] is first derivate [2] is second derivate*/
std_msgs::Float32MultiArray pd;
std_msgs::Float32MultiArray pd_d;
std_msgs::Float32MultiArray Rr;
std_msgs::Float32MultiArray agvr;
std_msgs::Float32MultiArray phi;
std_msgs::Int16 traj;

float t = 0;
enum {
    HOVERING_GRIPPER_STATIC,
    HOVERING_GRIPPER_SCISSORS,
    LAND,
}TRAJECTORY;


void trajectory_mode_cb(const std_msgs::Int16::ConstPtr& msg)
{
    traj = *msg;
    
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory_generator");
    ros::NodeHandle nh;
    /*initial some variable*/
    pd.data.resize(3);
    pd_d.data.resize(3);
    Rr.data.resize(3);
    agvr.data.resize(3);
    phi.data.resize(3);

    ros::Subscriber trajectory_mode = nh.subscribe<std_msgs::Int16>
        ("/system/trajectory",10,trajectory_mode_cb); 
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

    ros::topic::waitForMessage<std_msgs::Int16>("system/trajectory");
 
    ros::Time last_request = ros::Time::now();

    while(ros::ok() && ros::Time::now()-last_request<ros::Duration(0.3))
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::topic::waitForMessage<std_msgs::Bool>("/MAV/takeoff");
    ROS_WARN("START_GENERATE_TRAJECTORY");

    switch (traj.data)
    {
        case HOVERING_GRIPPER_STATIC:
            ROS_WARN("TRAJECTORY:HARVING(STATIC)");
            
            while(ros::ok())
            {   /*platform*/
                pd.data[0] = 0;
                pd.data[1] = 0;
                pd.data[2] = 0;

                pd_d.data[0] = 0;
                pd_d.data[1] = 0;
                pd_d.data[2] = 0;

                Rr.data[0] = 0;
                Rr.data[1] = 0;
                Rr.data[2] = 0;

                agvr.data[0] = 0;
                agvr.data[1] = 0;
                agvr.data[2] = 0;

                /*gripper*/
                phi.data[1] = 0;
                phi.data[2] = 0;
                desire_position.publish(pd);
                desire_velocity.publish(pd_d);
                desire_attitude.publish(Rr);
                desire_omega.publish(agvr);

                ros::spinOnce();
                rate.sleep();
            }
            break;
        case HOVERING_GRIPPER_SCISSORS:
            ROS_WARN("TRAJECTORY:HARVING(SCR)");
            while(ros::ok() )
            {
                pd.data[0] = 0;
                pd.data[1] = 0;
                pd.data[2] = 0;

                pd_d.data[0] = 0;
                pd_d.data[1] = 0;
                pd_d.data[2] = 0;

                Rr.data[0] = 0;
                Rr.data[1] = 0;
                Rr.data[2] = 0;

                agvr.data[0] = 0;
                agvr.data[1] = 0;
                agvr.data[2] = 0;

                /*gripper*/
                phi.data[0] = 0.1*sin(t)+ 90*D2R;
                phi.data[1] = 0.1*cos(t);
                phi.data[2] = -0.1*sin(t);

                desire_position.publish(pd);
                desire_velocity.publish(pd_d);
                desire_attitude.publish(Rr);
                desire_omega.publish(agvr);
                t += dt;
                ros::spinOnce();
                rate.sleep();
            }
            break;
    }
   
    return 0;
}