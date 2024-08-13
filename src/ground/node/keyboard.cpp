/*include ros.h*/
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "getch.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Mavlink.h"


std_msgs::Int16 Mode;

enum MAV_mod{
    IDLE,
    TAKEOFF,
    LAND,
};

class MAV_State
{
private:
    ros::Subscriber MAV1,MAV2,MAV3,MAV4; /*using check the wheater the MAV is arm*/

    mavros_msgs::State MAV1_State,MAV2_State,MAV3_State,MAV4_State;
public:
    MAV_State(ros::NodeHandle *nh);
    void MAV1_CB(const mavros_msgs::State::ConstPtr& msg);
    void MAV2_CB(const mavros_msgs::State::ConstPtr& msg);
    void MAV3_CB(const mavros_msgs::State::ConstPtr& msg);
    void MAV4_CB(const mavros_msgs::State::ConstPtr& msg);
    void Check_All_State(void);
    

};

MAV_State::MAV_State(ros::NodeHandle *nh)
{
    MAV1 = nh->subscribe("/MAV1/mavros/state", 10, &MAV_State::MAV1_CB,this);
    MAV2 = nh->subscribe("/MAV2/mavros/state", 10, &MAV_State::MAV2_CB,this);
    MAV3 = nh->subscribe("/MAV3/mavros/state", 10, &MAV_State::MAV3_CB,this);
    MAV4 = nh->subscribe("/MAV4/mavros/state", 10, &MAV_State::MAV4_CB,this);
}

void MAV_State::MAV1_CB(const mavros_msgs::State::ConstPtr& msg){
    MAV1_State = *msg;
}
void MAV_State::MAV2_CB(const mavros_msgs::State::ConstPtr& msg){
    MAV2_State = *msg;
}
void MAV_State::MAV3_CB(const mavros_msgs::State::ConstPtr& msg){
    MAV3_State = *msg;
}
void MAV_State::MAV4_CB(const mavros_msgs::State::ConstPtr& msg){
    MAV4_State = *msg;
}

void MAV_State::Check_All_State(void){
    if(MAV1_State.armed && MAV2_State.armed  && MAV4_State.armed && Mode.data == MAV_mod::IDLE){
        ROS_INFO("All_Mav_Get_Ready!!!");
    }
    if(Mode.data == MAV_mod::TAKEOFF){
        ROS_INFO("All_Mav_Takeoff!!!");
    }
    if(Mode.data == MAV_mod::LAND){
        ROS_INFO("All_Mav_Land!!!");
    }
}

int main(int argv,char** argc)
{

    ros::init(argv,argc,"keyboard");
    ros::NodeHandle nh;

    int c_prev = EOF;
    Mode.data = MAV_mod::IDLE;

    ros::Publisher SET_MODE = nh.advertise<std_msgs::Int16>("/ground_station/set_mode",10);


    MAV_State mav_state(&nh);

    ros::Rate rate(100.0);
    while(ros::ok())
    {
        mav_state.Check_All_State();
       
        int c = getch();

        if(c != EOF){
            switch (c)
            {
            case 'T':
                {
                    Mode.data = MAV_mod::TAKEOFF;
                    SET_MODE.publish(Mode);

                }
                break;
            case 'L':
                {
                    Mode.data = MAV_mod::LAND;
                    SET_MODE.publish(Mode);
                }
                break;

            }
            }
        c_prev = c;
        ros::spinOnce();
        rate.sleep();
        }


    return 0;
}
