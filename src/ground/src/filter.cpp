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

Eigen::Vector3d filter::Butterworth_filter(geometry_msgs::PoseStamped pose,int t){
    if(t == 0){
       position_raw_t_1 << 0,0,0;
       position_raw_t_2 << 0,0,0;
       position_raw_t_3 << 0,0,0;

       position_t_1 << 0,0,0;
       position_t_2 <<0,0,0;
       position_t_3 << 0,0,0;
    }

    position_raw << pose.pose.position.x,pose.pose.position.y,pose.pose.position.z;


    filt = b0*position_raw+b1*position_raw_t_1+b2*position_raw_t_2+b3*position_raw_t_3\
           -a1*position_t_1-a2*position_t_2-a3*position_t_3;

    /*renew the prev data*/
    position_t_3 = position_t_2;
    position_t_2 = position_t_1;
    position_t_1 = filt;

    /*renew the be failt data*/
    position_raw_t_3 = position_raw_t_2;
    position_raw_t_2 = position_raw_t_1;
    position_raw_t_1 = position_raw;
    
    

    return filt;
}