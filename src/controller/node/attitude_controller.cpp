#include "ros/ros.h"

#include "geometry_msgs/TwistStamped" //attitude velocity msgs
#include "geometry_msgs/PoseStamped" //attitude msgs

#include "Eigen/Dense"


using namespace Eigen ;
//attitude control gains
Matrix<float, 3, 3> KR;
Matrix<float, 3, 3> Kw;
Matrix<float, 3, 3> Ki;

Matrix<float, 3, 3> E;
Matrix<float, 3, 3> eR;
Matrix<float, 3, 3> eW;
Matrix<float, 3, 3> eRiout;

KR << 17.0, 0 , 0 ,
      0  , 15.0,0
      0  , 0,  8.0;

Kw << 6.0, 0 , 0 ,
      0  , 5.5,0
      0  , 0,  3.0;

Ki << 5.5, 0 , 0 ,
      0  , 4.5,0
      0  , 0,  3;


// 


int main(int argc,char **argv)
{
    ros::init(argc,argv,"attitude_controller");
    ros::NodeHandle nh;
    ros::Subscriber desire_agvr = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/desire_agvr",10,desire_agvr_cb);
    ros::Subscriber desire_agvr = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/measure_attitude",10,measure_attitude_cb);
    ros::Subscriber desire_agvr = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/measure_attitude",10,measure_attitude_cb);
    ros::Subscriber desire_agvr = nh.subscribe<std_msgs::Float32MultiArray>
        ("gripper/measure_attitude",10,measure_attitude_cb);                                                

    ros::Rate rate(100);

    while(ros::ok)
    {



        ros::spinOnce();

        rate.sleep();
    }




    return 0;
}