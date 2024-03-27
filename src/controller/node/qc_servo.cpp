#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "cmath"
#include "string"


#define six -3.0607845945e-8
#define fith 2.8310812334e6
#define fourth -1.0243476854e-4
#define third 1.8488037507e-3
#define second -1.8071097239e-2
#define first 1.2644148433e-1
#define zero 3.2072458244e-2

using namespace Eigen;
using namespace std;

Quaterniond q;

      
std_msgs::Float64MultiArray fd; // desire forc for each UAV

class MAV{
    private:
        std_msgs::Float64MultiArray T; //T[0] is net thrust T[1] is apha T[2] is beta
        Vector3f fd_e;
        ros::Publisher MAV_cmd ;#include "std_msgs/Bool.h"

    public:
        MAV(ros::NodeHandle nh, string Topic);
        void Thrust(std_msgs::Float64MultiArray \
            fd,int i);

};

MAV::MAV(ros::NodeHandle nh, string Topic)
{
    MAV_cmd = nh.advertise<std_msgs::Float64MultiArray>(Topic,10);

}

void MAV::Thrust(std_msgs::Float64MultiArray fd,int i)
{

    fd_e(0,1) = fd.data[i];
    fd_e(1,1) = fd.data[i+1];
    fd_e(2,1) = fd.data[i+2];
    
    T.data[0] = fd_e.norm(); // net thrust(PWM 0~1) you should imply thrust curve here
    q = AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
        AngleAxisd(asin(fd_e(0,1)/T.data[0]), Eigen::Vector3d::UnitY()) *
        AngleAxisd(atan2(-fd_e(1,1),fd_e(2,1)), Eigen::Vector3d::UnitX());
    T.data[1] = q.w();
    T.data[2] = q.x();
    T.data[3] = q.y();
    T.data[4] = q.z();

    MAV_cmd.publish(T);
}

void thrust_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    fd = *msg;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"qc_servo");
    ros::NodeHandle nh;

    ros::Subscriber thrust = nh.subscribe<std_msgs::Float64MultiArray>
        ("/gripper/desire_thrust_each",10,thrust_cb);

    MAV mav[4] = {MAV(nh, "/MAV1/cmd"),
                  MAV(nh, "/MAV2/cmd"),
                  MAV(nh, "/MAV3/cmd"),
                  MAV(nh, "/MAV4/cmd")};
                  
    ros::Rate rate(100);
    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/gripper/desire_thrust_each");

    while(ros::ok())
    {
        mav[0].Thrust(fd,0);
        mav[1].Thrust(fd,3);
        mav[2].Thrust(fd,6);
        mav[3].Thrust(fd,9);
        
        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}