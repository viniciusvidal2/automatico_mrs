#include "ros/ros.h"
#include "std_msgs/String.h"

//#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
//#include <mavlink/config.h>
#include <mavros/mavros.h>
#include <mavros_msgs/AttitudeTarget.h>

void chatterCallback(const mavros_msgs::AttitudeTargetConstPtr& msg)
{
    ROS_INFO("I heard: [%.2f]", msg->body_rate.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controle_automatico");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/mavlink/from", 1000, chatterCallback);

    ros::spin();

    return 0;
}
