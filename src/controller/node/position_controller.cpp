#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "Eigen/Dense"
using namespace Eigen;

std_msgs::Float32MultiArray p;
std_msgs::Float32MultiArray pd;
std_msgs::Float32MultiArray p_dot;
std_msgs::Float32MultiArray pd_dot;
std_msgs::Float32MultiArray u1;

p.data.resize(3);
pd.data.resize(3);
p_dot.data.resize(3);
pd_dot.data.resize(3);
u1.data.resize(3);


Vector3f ep;
Vector3f ev;
Vector3f epiout;
Vector3f epiin;

Matrix<float, 3, 3> Kp;
Matrix<float, 3, 3> Kv;
Matrix<float, 3, 3> Ki;

//control gain
Kp << 4.5, 0 , 0 ,
      0  , 4.5,0
      0  , 0,  5;

Kv << 2  , 0  ,0 ,
      0  , 2  ,0,
      0  , 0  ,2.5;

Ki << 2.7, 0 , 0 ,
      0  , 2.5,0,
      0  , 0,  3;


void desire_position_cb(const )
void desire_velocity_cb
void measure_position_cb
void measure_velocity_cb

int main(int argc,char **argv)
{
    ros::init(argc,argv,"position_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber desire_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_position",10,desire_position_cb); 

    ros::Subscriber desire_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/desire_velocity",10,desire_velocity_cb);

    ros::Subscriber measure_position = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_position",10,measure_position_cb); 

    ros::Subscriber measure_velocity = nh.subscribe<std_msgs::Float32MultiArray>
        ("platform/measure_velocity",10,measure_velocity_cb); 

    ros::Publisher desire_thrust_total = nh.advertise<std_msgs::Float32MultiArray>
        ("platform/desire_thrust_total",10,);

    ros::Rate rate(100);
    while(ros::ok())
    {

        total_thrust();
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}