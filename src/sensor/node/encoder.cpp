#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "string"
#include "sstream"
#include "vector"


using namespace std;

string data= "@0.0,0.0,0.0;";
string str1,str2,str3;
int start = 1;
int ended = 2;

std_msgs::Float32MultiArray value;


/*receive encoder_value string from Rpi 4B*/
void encoder_cb(const std_msgs::String::ConstPtr &msg)
{
    data = msg->data;
}

int str2float()
{
    if(data == "*"){
        return 0;
    }
    if(data[0] != '@'){
        return 0;
    }
    start = data.find('@');
    ended = data.find(',');
    str1 = data.substr(start+1,ended-1);
    start = ended;
    ended = data.find(',',start+1);
    str2 = data.substr(start+1,ended-1);
    start = ended;
    ended = data.find(';');
    str3 = data.substr(start+1,ended-1);
    value.data[0] = stof(str1);
    value.data[1] = stof(str2);
    value.data[2] = stof(str3);
    //cout<<str1<<"\n";
    //cout<<str2<<"\n";
    //cout<<str3<<"\n";
    //cout<<str2;
    //cout<<str3;

    

    return 0;
}

int main(int argv,char **argc)
{
    ros::init(argv,argc,"encoder");
    ros::NodeHandle nh;
    value.data.resize(3);    
    ros::Subscriber encoder_str = nh.subscribe<std_msgs::String>
        ("/encoder_str",10,encoder_cb);
    ros::Publisher encoder_float = nh.advertise<std_msgs::Float32MultiArray>
        ("/encoder_value",10);     


    ros::Rate rate(100);
    while(ros::ok())
    {
        str2float();
        encoder_float.publish(value);
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}