 /*This is a Butterworth filter .h file the coefficient using matlab to get*/
#include "filter.h"


filter::filter(Eigen::Vector3d a,Eigen::Vector4d b){
    a1 =a(0);
    a2 =a(1);
    a3 =a(2);
    b0 =b(0);
    b1 =b(1);
    b2 =b(2);
    b3 =b(3);


}

Eigen::Vector3d filter::Butterworth_filter_position(geometry_msgs::PoseStamped pose,int t){
    if(t == 0){
       position_raw_t_1 << 0,0,0;
       position_raw_t_2 << 0,0,0;
       position_raw_t_3 << 0,0,0;

       position_t_1 << 0,0,0;
       position_t_2 <<0,0,0;
       position_t_3 << 0,0,0;
    }

    position_raw << pose.pose.position.x,pose.pose.position.y,pose.pose.position.z;


    //filt_position = b0*position_raw+b1*position_raw_t_1+b2*position_raw_t_2+b3*position_raw_t_3\
           -a1*position_t_1-a2*position_t_2-a3*position_t_3;
        
    filt_position = b0*position_raw+b1*position_raw_t_1\
           -a1*position_t_1;

    

    /*renew the prev data*/
   // position_t_3 = position_t_2;
   // position_t_2 = position_t_1;
    position_t_1 = filt_position;

    /*renew the be failt data*/
    //position_raw_t_3 = position_raw_t_2;
   // position_raw_t_2 = position_raw_t_1;
    position_raw_t_1 = position_raw;
    
    

    return filt_position;
}


Eigen::Vector4d filter::Butterworth_filter_attitude(geometry_msgs::PoseStamped pose,int t){
    if(t == 0){
       attitude_raw_t_1 << 1,0,0,0;
       attitude_raw_t_2 << 1,0,0,0;
       attitude_raw_t_3 << 1,0,0,0;

       attitude_t_1 << 1,0,0,0;
       attitude_t_2 << 1,0,0,0;
       attitude_t_3 << 1,0,0,0;
    }

    attitude_raw << pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z;


  // filt_attitude = b0*attitude_raw+b1*attitude_raw_t_1+b2*attitude_raw_t_2+b3*attitude_raw_t_3\
           -a1*attitude_t_1-a2*attitude_t_2-a3*attitude_t_3;

    filt_attitude = b0*attitude_raw+b1*attitude_raw_t_1+b2*attitude_raw_t_2\
           -a1*attitude_t_1-a2*attitude_t_2;

    /*renew the prev data*/
    //attitude_t_3 = attitude_t_2;
    attitude_t_2 = attitude_t_1;
    attitude_t_1 = filt_attitude;

    /*renew the be failt data*/
   // attitude_raw_t_3 = attitude_raw_t_2;
    attitude_raw_t_2 = attitude_raw_t_1;
    attitude_raw_t_1 = attitude_raw;
    
    
    return filt_attitude;
}


Eigen::Vector3d filter::Butterworth_filter_angular_rate(geometry_msgs::Vector3Stamped imu_data,int t){
       if(t == 0){
       imu_angular_rate_raw_t_1 << 0,0,0;
       imu_angular_rate_raw_t_2 << 0,0,0;
  
       imu_angular_rate_t_1 << 0,0,0;
       imu_angular_rate_t_2 << 0,0,0;
       }

    imu_angular_rate_raw << imu_data.vector.x,imu_data.vector.y,imu_data.vector.z;


    filt_angular_rate = b0*imu_angular_rate_raw+b1*imu_angular_rate_raw_t_1+b2*imu_angular_rate_raw_t_2\
          -a1*imu_angular_rate_t_1-a2*imu_angular_rate_t_2;

    /*renew the prev data*/
  
    imu_angular_rate_t_2 = imu_angular_rate_t_1;
    imu_angular_rate_t_1 = filt_angular_rate;

    /*renew the be failt data*/
 
    imu_angular_rate_raw_t_2 = imu_angular_rate_raw_t_1;
    imu_angular_rate_raw_t_1 =  imu_angular_rate_raw;
    
    
    return filt_angular_rate;


}