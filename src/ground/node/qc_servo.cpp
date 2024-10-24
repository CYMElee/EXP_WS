#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "cmath"
#include "string"



// define the motor curve index
double p7 = 0.03169;
double p6 = 0.2807;
double p5 = -0.07407;
double p4 = 0.01128;
double p3 = -0.0007524;
double p2 = 1.375e-05;
double p1 = 3.274e-07;




using namespace Eigen;
using namespace std;

Quaterniond q;

      
std_msgs::Float64MultiArray fd; // desire forc for each UAV

class MAV{
    private:
        std_msgs::Float64MultiArray T; //T[0] is net thrust T[1] is apha T[2] is beta
        Vector3d fd_e;
        ros::Publisher MAV_cmd;

    public:
        MAV(ros::NodeHandle nh, string Topic);
        void Thrust(std_msgs::Float64MultiArray \
            fd,int i);

};

MAV::MAV(ros::NodeHandle nh, string Topic)
{
    T.data.resize(3);
    MAV_cmd = nh.advertise<std_msgs::Float64MultiArray>(Topic,10);

}

void MAV::Thrust(std_msgs::Float64MultiArray fd,int i)
{

    fd_e(0) = fd.data[i]; //x
    fd_e(1) = fd.data[i+1]; //y
    fd_e(2) = fd.data[i+2]; //z
    double f = (fd_e.norm()/4); //because we have 4 motor for each sd420

    T.data[0] = p1*pow(f,6)+p2*pow(f,5)+p3*pow(f,4)+p4*pow(f,3)+p5*pow(f,2)+p6*pow(f,1)+p7;   // net thrust(PWM 0~1) you should imply thrust curve here
    if (T.data[0]>=1.0)
            T.data[0] = 1.0;


    /*using the desire thrust(vector) on platform body frame to get the alpha and beta*/

    


    double alpha = atan2(-fd_e(1),fd_e(2));
    double beta = asin(fd_e(0)/fd_e.norm());
    
  

    T.data[1] = alpha;
    T.data[2] = beta;


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

    fd.data.resize(12);
    ROS_INFO("SUCCESS LAUNCH QC_SERVO!!"); 
    ros::Subscriber thrust = nh.subscribe<std_msgs::Float64MultiArray>
        ("/plarform/desire_thrust_each",10,thrust_cb);

    MAV mav[4] = {MAV(nh, "/MAV1/cmd"),
                  MAV(nh, "/MAV2/cmd"),
                  MAV(nh, "/MAV3/cmd"),
                  MAV(nh, "/MAV4/cmd")};
                  
    ros::Rate rate(100);
    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/plarform/desire_thrust_each");

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