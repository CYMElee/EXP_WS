#include "ros/ros.h"

#include "geometry_msgs/TwistStamped" //attitude velocity msgs
#include "geometry_msgs/PoseStamped" //attitude msgs
#include "std_msgs/Float32MultiArray.h"

#include "Eigen/Dense"
#include "constant.h"

using namespace Eigen ;

Platform platform;

std_msgs::Float32MultiArray u2;

Matrix<float, 3, 3> R; //measure attitude
Matrix<float, 3, 3> Rr; // desire attitude
RowVector3f agvr;
RowVector3f omega;
RowVector3f m;

RowVector3f h;


//attitude control gains
Matrix<float, 3, 3> KR;
Matrix<float, 3, 3> Kw;
Matrix<float, 3, 3> Ki;

Matrix<float, 3, 3> E;
Vector3f eR;
Vector3f eW;

KR << 17.0, 0 , 0 ,
      0  , 15.0,0
      0  , 0,  8.0;

Kw << 6.0, 0 , 0 ,
      0  , 5.5,0
      0  , 0,  3.0;

Ki << 5.5, 0 , 0 ,
      0  , 4.5,0
      0  , 0,  3;


void desire_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Eigen::AngleAxisd roll_angle(msg->data[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(msg->data[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(msg->data[2], Eigen::Vector3d::UnitZ());
    Rr = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    


}

void measure_attitude_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Eigen::AngleAxisd roll_angle(msg->data[0] , Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(msg->data[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(msg->data[2], Eigen::Vector3d::UnitZ());
    R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();


}

void desire_omega_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    agvr(0) = msg->data[0];
    agvr(1) = msg->data[1];
    agvr(2) = msg->data[2];


}

void measure_omega_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    omega(0) = msg ->data[0];
    omega(1) = msg ->data[1];
    omega(2) = msg ->data[2];

    


}

void attitude_er()
{
    E = 0.5*(Rr.transpose*R-R.transpose*Rr);
    eR(0) = E(2,1);
    eR(1) = E(0,2);
    eR(2) = E(1,0);
    eW = omega - R.transpose*(Rr*agvr);

    //Physical dynamic to cancel
    h = omega.cross(platform.IB*omega);
}

void moment()
{
    m = h - platform.IB*(KR*eR + Kw*eW );

    u2.data[0] = m(0);
    u2.data[1] = m(1);
    u2.data[2] = m(2);

}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"attitude_controller");
    ros::NodeHandle nh;
    ros::Subscriber desire_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_attitude",10,desire_attitude_cb);
    ros::Subscriber measure_attitude = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_attitude",10,measure_attitude_cb);
    ros::Subscriber desire_angular_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_omega",10,desire_omega_cb);
    ros::Subscriber measure_omega = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_omega_cb",10,measure_omega_cb);  
    ros::Publisher total_moment = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/desire_thrust_total",10,);                                              

    ros::Rate rate(100);

    while(ros::ok)
    {
        ros::spinOnce();
        
        attitude_er();

        h();

        moment();

        total_moment.publish(u2);


        rate.sleep();
    }




    return 0;
}