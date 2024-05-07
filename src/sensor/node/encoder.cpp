#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"



int main(int argv,char **argc)
{

    ros::Rate rate(100);
    while(ros::ok())
    {


        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}