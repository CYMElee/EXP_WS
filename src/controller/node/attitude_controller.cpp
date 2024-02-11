#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Dense"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"attitude_controller");
    ros::NodeHandle nh;


    ros::Rate rate(100);

    while(ros::ok)
    {




        ros::spinOnce();

        rate.sleep();
    }




    return 0;
}