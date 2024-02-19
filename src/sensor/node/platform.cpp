#include "ros/ros.h"

#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PointStamped.h"
#define dt 0.01

float vel;
float vel_prev;

geometry_msgs::PointStamped pose;
geometry_msgs::PointStamped pose_prev;



std_msgs::Int32MultiArray position;
std_msgs::Int32MultiArray velocity;



void platform_pose_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    pose = *msg;

}

void position()
{
    position[0] = pose.pose.position.x;
    position[1] = pose.pose.position.y;
    position[2] = pose.pose.position.z;
}


void velocity()
{    
    velocity[0] = (pose.pose.position.x - pose_prev.pose.position.x)/dt;
    velocity[1] = (pose.pose.position.y - pose_prev.pose.position.y)/dt;
    velocity[2] = (pose.pose.position.z - pose_prev.pose.position.z)/dt;

}

void attitude()
{


}

void angular_velocity()
{



}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"platform_position");
    ros::NodeHandle nh;
    ros::Subscriber platform_pose = nh.subscribe<geometry_msgs::PointStamped>
        ("/vrpn_client_node/Platform/pose",10,platform_pose_cb);
    
    ros::Publisher platform_position = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_position",10);
    ros::Publisher platform_position = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_velocity",10);
    ros::Publisher platform_position = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_attitude",10);
    ros::Publisher platform_position = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/measure_angular_velocity",10);


    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        position();
        velocity();
        attitude();
        angular_velocity();
        pose_prev = pose;
        rate.sleep();
    }

    return 0;
}