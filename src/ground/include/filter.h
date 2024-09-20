#ifndef FILTER_H
#define FILTER_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

class filter
{
private:
    double a1,a2,a3;
    double b0,b1,b2,b3;

    /*using for position filter*/
    Eigen::Vector3d position_raw_t_1; 
    Eigen::Vector3d position_raw_t_2; 
    Eigen::Vector3d position_raw_t_3;

    Eigen::Vector3d position_t_1; 
    Eigen::Vector3d position_t_2; 
    Eigen::Vector3d position_t_3;

    Eigen::Vector3d position_raw;

    Eigen::Vector3d filt_position;
     /*using for attitude filter*/

    Eigen::Vector4d attitude_raw_t_1; 
    Eigen::Vector4d attitude_raw_t_2; 
    Eigen::Vector4d attitude_raw_t_3;

    Eigen::Vector4d attitude_t_1; 
    Eigen::Vector4d attitude_t_2; 
    Eigen::Vector4d attitude_t_3;

    Eigen::Vector4d attitude_raw;

    Eigen::Vector4d filt_attitude;


        /*using for imu data*/
    Eigen::Vector3d imu_angular_rate_raw;
    Eigen::Vector3d imu_angular_rate_raw_t_1; 
    Eigen::Vector3d imu_angular_rate_raw_t_2; 

    Eigen::Vector3d imu_angular_rate_t_1;
    Eigen::Vector3d imu_angular_rate_t_2;
    Eigen::Vector3d filt_angular_rate;

    

public:
    filter(Eigen::Vector3d a,Eigen::Vector4d b);
    Eigen::Vector3d Butterworth_filter_position(geometry_msgs::PoseStamped pose,int t);
    Eigen::Vector4d Butterworth_filter_attitude(geometry_msgs::PoseStamped pose,int t);
    Eigen::Vector3d Butterworth_filter_angular_rate(geometry_msgs::Vector3Stamped imu_data,int t);
  
};
#endif
