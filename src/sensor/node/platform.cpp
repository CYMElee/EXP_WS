#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Dense"

#define dt 0.01;

using namespace Eigen;

RowVector3f p;
RowVector3f p_prev;

RowVector3f eu;
RowVector3f eu_prev;

Matrix3f R;


std_msgs::Float32MultiArray PLA_p;   //platform position
std_msgs::Float32MultiArray PLA_p_d; //platform velocity
std_msgs::Float32MultiArray PLA_r;   //platform attitude
std_msgs::Float32MultiArray PLA_agvr;//platform omga

geometry_msgs::PoseStamped pose;

Quaternionf quaternion;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose = *msg;
}

void position(void)
{
    PLA_p.data[0] = pose.pose.position.x;
    PLA_p.data[1] = pose.pose.position.y;
    PLA_p.data[2] = pose.pose.position.z;
    p[0] = pose.pose.position.x;
    p[1] = pose.pose.position.y;
    p[2] = pose.pose.position.z;

}

void velocity(void)
{
    PLA_p_d.data[0] = (p[0]-p_prev[0])/dt ;
    PLA_p_d.data[1] = (p[1]-p_prev[1])/dt ;
    PLA_p_d.data[2] = (p[2]-p_prev[2])/dt ;

}

void attitude(void)
{
    PLA_r.data[0] = pose.pose.orientation.w;
    PLA_r.data[1] = pose.pose.orientation.x;
    PLA_r.data[2] = pose.pose.orientation.y;
    PLA_r.data[3] = pose.pose.orientation.z;

    //transfer the attitude from Orientation to Euler

    R = quaternion.toRotationMatrix();
    eu = R.eulerAngles(0,1,2);


}


void omega(void)
{
    PLA_agvr.data[0] = (eu[0] - eu_prev[0])/dt;
    PLA_agvr.data[1] = (eu[1] - eu_prev[1])/dt;
    PLA_agvr.data[2] = (eu[2] - eu_prev[0])/dt;

}



int main(int argv,char **argc)
{
    ros::init(argv,argc,"platform");
    ros::NodeHandle nh;

    ros::Subscriber pla_pose = nh.subscribe<geometry_msgs::PoseStamped>
        ("/platform/measure_pose",10,pose_cb);

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

        pos_pub.publish(PLA_p);
        vel_pub.publish(PLA_p_d);
        attitude_pub.publish(PLA_r);
        omega_pub.publish(PLA_agvr);

        p_prev = p;
        eu_prev = eu;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}