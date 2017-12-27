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
    // Dados vindos da placa
    float pitch_para_apontar, yaw_atual, yaw_para_apontar;
    // Ranges para alcance de pwm e angulo [DEGREES] de yaw e pitch
    int pwm_yaw_range[2]     = {0, 1023};
    int pwm_pitch_range[2]   = {1437, 2652}; // TODO: achar limites certos aqui
    float ang_yaw_range[2]   = {0, 300}; // TODO: achar limites certos aqui
    float ang_pitch_range[2] = {0, 330}; // TODO: achar limites certos aqui
    float yaw_mid_range, pitch_mid_range; // [DEGREES]
    // Relacao pwm/ang[DEGREES] para os dois casos
    float pwm_ang_yaw, pwm_ang_pitch;
    // Diferenca entre angulos de yaw e pitch
    float delta_yaw, delta_pitch;
    // Posicoes para os motores de yaw e tilt
    float ang_pan, ang_tilt; // PAN esta para YAW e TILT para PITCH
    // ENtidades do ROS
    ros::NodeHandle nh_;
    ros::Subscriber subPix;
    ros::ServiceClient motor;

public:
    PixhawkeMotor()
    {
        // Inicia a escuta do topico ja publicado de chegada da mensagem
        subPix = nh_.subscribe("/mavros/vfr_hud", 10, &PixhawkeMotor::escutarPixhawk, this);
        // Inicia a relacao de pwm/ang para cada motor
        pwm_ang_yaw = (pwm_yaw_range[1] - pwm_yaw_range[0]) / (ang_yaw_range[1] - ang_yaw_range[0]);         // [PWM/DEGREES]
        pwm_ang_yaw = (pwm_pitch_range[1] - pwm_pitch_range[0]) / (ang_pitch_range[1] - ang_pitch_range[0]); // [PWM/DEGREES]
        // Pontos centrais dos dois servos
        yaw_mid_range   = (ang_yaw_range[1]   - ang_yaw_range[0])  /2; // [DEGREES]
        pitch_mid_range = (ang_pitch_range[1] - ang_pitch_range[0])/2; // [DEGREES]
    }

    int ExecutarClasse(int argc, char **argv)
    {

    }

private:
    void calcularAngulosMotores()
    {
        // Analisando diferenca de yaw
        delta_yaw = wrap180(yaw_atual, yaw_para_apontar);
        ang_pan = ((yaw_mid_range + delta_yaw) < ang_yaw_range[1]) ? yaw_mid_range + delta_yaw : ang_yaw_range[1]; // LImitando maximo
        ang_pan = (ang_pan > ang_yaw_range[0]) ? ang_pan : ang_yaw_range[0]; // LImitando minimo



    }

    float rad2deg(float rad)
    {
        return 180.0/3.1415926535 * rad;
    }

    // Mantem o angulo de diferenca entre 180 da forma mais logica no algoritmo
    float wrap180(float atual, float apontar)
    {
        float delta = atual - apontar;
        if(delta >  180) {delta -= 360.0;}
        if(delta < -180) {delta += 360.0;}

        return delta;
    }

    void escutarPixhawk(const mavros_msgs::VFR_HUDConstPtr& msg)
    {
        /// Relacoes obtidas da mensagem VFR_HUD vinda da placa
        pitch_para_apontar = rad2deg(msg->airspeed);     // [RAD] -> [DEGREES]
        yaw_atual          = msg->groundspeed;           // [DEGREES]
        // Ajusta chegada desse angulo que vai de -180 a +180
        yaw_para_apontar   = ((float)(msg->heading)*0.01 >= 0) ? (float)(msg->heading)*0.01 : (float)(msg->heading)*0.01 + 360.0; // [DEGREES]

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
