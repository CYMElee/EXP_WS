#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"

std_msgs::Float32MultiArray PLA_p;   //platform position
std_msgs::Float32MultiArray PLA_p_d; //platform velocity
std_msgs::Float32MultiArray PLA_r;   //platform attitude
std_msgs::Float32MultiArray PLA_agvr;//platform omga

geometry_msgs::PoseStamped pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose = *msg;

}

void position()
{
    PLA_p.data[0] = pose.pose.position.x;
    PLA_p.data[1] = pose.pose.position.y;
    PLA_p.data[2] = pose.pose.position.z;

}

void velocity()
{



}

void attitude()
{
    PLA_r.data[0] = pose.pose.orientation.w;
    PLA_r.data[1] = pose.pose.orientation.x;
    PLA_r.data[2] = pose.pose.orientation.y;
    PLA_r.data[3] = pose.pose.orientation.z;

}


void omega()
{



}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"platform");
    ros::NodeHandle nh;

    ros::Subscriber pla_pose = nh.subscribe<geometry_msgs::PoseStamped>
        ("/platform/measure_pose",pose_cb,10);

    ros::Publisher pos_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_position",10);

    ros::Publisher vel_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10);

    ros::Publisher attitude_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10);

    ros::Publisher omega_pub =  nh.advertise<std_msgs::Float32MultiArray>
        ("/platform/measure_velocity",10);






    ros::Rate rate(100);

    while(ros::ok())
    {
        position();
        velocity();
        attitude();
        omega();


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}