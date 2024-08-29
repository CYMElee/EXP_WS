#ifndef FILTER_H
#define FILTER_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class filter
{
private:
    double a1,a2,a3;
    double b0,b1,b2,b3;

    Eigen::Vector3d position_raw_t_1; 
    Eigen::Vector3d position_raw_t_2; 
    Eigen::Vector3d position_raw_t_3;

    Eigen::Vector3d position_t_1; 
    Eigen::Vector3d position_t_2; 
    Eigen::Vector3d position_t_3;

    Eigen::Vector3d position_raw;

    Eigen::Vector3d filt;


public:
    filter(Eigen::Vector3d a,Eigen::Vector4d b);
    Eigen::Vector3d Butterworth_filter(geometry_msgs::PoseStamped pose,int t);
    /*the return value [x,y,z] that be filter   */
    /*the input value [x_raw,y_raw,z_raw]       */
};
#endif
