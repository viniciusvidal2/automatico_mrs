#include "ros/ros.h"
#include "std_msgs/String.h"

#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/config.h>
#include <mavros/utils.h>
#include <mavros/mavros.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/VFR_HUD.h>

class PixhawkeMotor
{
private:
    float pitch_para_apontar, yaw_atual, yaw_para_apontar;
    ros::NodeHandle nh_;
    ros::Subscriber subPix;

public:
    PixhawkeMotor()
    {
        subPix = nh_.subscribe("/mavros/vfr_hud", 10, &PixhawkeMotor::escutarPixhawk, this);
    }
    int ExecutarClasse(int argc, char **argv)
    {

    }

private:
    void escutarPixhawk(const mavros_msgs::VFR_HUDConstPtr& msg)
    {
        // Relacoes obtidas da mensagem VFR_HUD vinda da placa
        pitch_para_apontar = msg->airspeed;
        yaw_atual          = msg->groundspeed;
        yaw_para_apontar   = (float)(msg->heading)*0.01;

        // Mostrando na tela se esta tudo ok
        ROS_INFO("Pitch: [%.2f]", pitch_para_apontar);
        ROS_INFO("Yaw:   [%.2f]", yaw_atual);
        ROS_INFO("ALvo:  [%.2f]", yaw_para_apontar);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controle_automatico");
    PixhawkeMotor pxm;

    pxm.ExecutarClasse(argc, argv);

    ros::spin();

    return 0;
}
