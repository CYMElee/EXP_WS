#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Dense"


#define Lw_sq 0.64
double IBXY = 2*1.5*0.64;
double IBZ  = 4*1.5*0.64;

using namespace Eigen ;

double IBxy = IBXY;
double IBz = IBZ;

std_msgs::Float64MultiArray u2;
std_msgs::Float64MultiArray er;
Matrix<double, 3, 3> IB; //rotation inertia
Matrix<double, 3, 3> R; //measure attitude,which is rotation matrix that from body frame to inertial frame
Matrix<double, 3, 3> Rr; // desire attitude
Vector3d agvr;
Vector3d omega;
Vector3d m;
Vector3d h;


//attitude control gains
Matrix<double, 3, 3> KR;
Matrix<double, 3, 3> Kw;
Matrix<double, 3, 3> Ki;

Matrix<double, 3, 3> E;
Vector3d eR;
Vector3d eW;

Quaterniond quaternion;


void desire_attitude_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    quaternion = AngleAxisd(msg->data[0], \
    Vector3d(msg->data[1], msg->data[2], msg->data[3]));
    Rr = quaternion.toRotationMatrix();
    


}

void measure_attitude_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    quaternion = AngleAxisd(msg->data[0], \
    Vector3d(msg->data[1], msg->data[2], msg->data[3]));
    R = quaternion.toRotationMatrix();


}

void desire_omega_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    agvr(0) = msg->data[0];
    agvr(1) = msg->data[1];
    agvr(2) = msg->data[2];


}

void measure_omega_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    omega(0) = msg ->data[0];
    omega(1) = msg ->data[1];
    omega(2) = msg ->data[2];

    


}

void attitude_er()
{
    E = 0.5*(Rr.transpose()*R-R.transpose()*Rr);
    eR(0) = E(2,1);
    eR(1) = E(0,2);
    eR(2) = E(1,0);
    er.data[0] = eR(0);
    er.data[1] = eR(1);
    er.data[2] = eR(2);

    eW = omega -( R.transpose()*(Rr*agvr));

    //Physical dynamic to cancel
    h = omega.cross(IB*omega);
}

void moment()
{
    m =h - IB*(KR*eR + Kw*eW);

    u2.data[0] = m(0);
    u2.data[1] = m(1);
    u2.data[2] = m(2);

}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"attitude_controller");
    ros::NodeHandle nh;
    u2.data.resize(3);
    er.data.resize(3);
    KR << 4, 0 , 0 ,
          0 , 4, 0 ,
          0 , 0 , 4;

    Kw << 1  , 0 , 0 ,
          0  , 1, 0 ,
          0  , 0,  1;


    IB << IBxy,  0  , 0,
            0 , IBxy, 0,
            0 ,  0  , IBz;

    ros::Subscriber desire_attitude = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/desire_attitude",10,desire_attitude_cb);
    ros::Subscriber measure_attitude = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/measure_attitude",10,measure_attitude_cb);
    ros::Subscriber desire_omega = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/desire_omega",10,desire_omega_cb);
    ros::Subscriber measure_omega = nh.subscribe<std_msgs::Float64MultiArray>
        ("/platform/measure_omega",10,measure_omega_cb);  
    ros::Publisher total_moment = nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/desire_total_moment",10); 
    ros::Publisher attitude_error = nh.advertise<std_msgs::Float64MultiArray>
        ("/platform/attitude_error",10);

    ROS_INFO("SUCCESS LAUNCH ATTITUDE CONTROLLER"); 
 
    ros::Rate rate(100);
    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/platform/desire_attitude");
    while(ros::ok)
    {
        
        attitude_er();
        moment();

        total_moment.publish(u2);
        attitude_error.publish(er);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}