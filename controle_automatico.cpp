#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controle_automatico");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/mavlink/from", 1000, chatterCallback);

    ros::spin();

    return 0;
}
