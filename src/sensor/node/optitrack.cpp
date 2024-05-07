#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "Eigen/Dense"
#include "string"


using namespace std;


geometry_msgs::PoseStamped pla_pose;



class MAV{
    private:
        geometry_msgs::PoseStamped pose;
      
        ros::Publisher MAV_pose ;
        ros::Subscriber pose_sub ;


    public:
        MAV(ros::NodeHandle nh, string subTopic,string pubTopic);
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

MAV::MAV(ros::NodeHandle nh,string subTopic,string pubTopic)
{
    
    MAV_pose = nh.advertise<geometry_msgs::PoseStamped>(pubTopic,10);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &MAV::pose_cb, this);

}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   
  
    pose = *msg;
    MAV_pose.publish(pose);

}


void platform_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pla_pose = *msg;

}


int main(int argv,char **argc)
{
    ros::init(argv,argc,"optitrack");
    ros::NodeHandle nh;

    ros::Subscriber platform = nh.subscribe<geometry_msgs::PoseStamped>
        ("/vrpn_client_node/PLA/pose",10,platform_cb);

    ros::Publisher platform_pose = nh.advertise<geometry_msgs::PoseStamped>
        ("/platform/measure_pose",10);

    MAV mav[4] = {MAV(nh, "/vrpn_client_node/MAV1/pose","mavros/vision_pose/pose"),
                  MAV(nh, "/vrpn_client_node/MAV2/pose","mavros/vision_pose/pose"),
                  MAV(nh, "/vrpn_client_node/MAV3/pose","mavros/vision_pose/pose"),
                  MAV(nh, "/vrpn_client_node/MAV4/pose","mavros/vision_pose/pose")};

    ros::Rate rate(100);

    while(ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }

    return 0 ;
}


