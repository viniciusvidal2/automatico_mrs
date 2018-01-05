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

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

class PixhawkeMotor
{
private:
    // Dados vindos da placa
    float pitch_para_apontar, yaw_atual, yaw_para_apontar;
    // Ranges para alcance de pwm e angulo [DEGREES] de yaw e pitch
    int pwm_yaw_range[2]     = {0, 1023}; // [PWM]
    int pwm_pitch_range[2]   = {1655, 2435}; // [PWM]
    float ang_yaw_range[2]   = {0.0  , 300.0}; // [DEGREES]
    float ang_pitch_range[2] = {145.0, 214.0}; // [DEGREES]
    float ang_pitch_horizontal = 173.0; // [DEGREES] 1976 DE PWM
    int pwm_pitch_horizontal = 1976; // [PWM]
    int pwm_yaw_frente = 512; // apontar sempre para frente do veiculo caso nao precise virar [PWM]
    float yaw_mid_range, pitch_mid_range; // [DEGREES]
    // Relacao pwm/ang[DEGREES] para os dois casos
    float pwm_ang_yaw, pwm_ang_pitch;
    // Diferenca entre angulos de yaw e pitch
    float delta_yaw, delta_pitch;
    // Posicoes para os motores de yaw e tilt
    float ang_pan, ang_tilt; // PAN esta para YAW e TILT para PITCH [DEGREES]
    int   pwm_pan, pwm_tilt; // [PWM]
    // ENtidades do ROS
    ros::NodeHandle nh_;
    ros::Subscriber subPix;
    ros::ServiceClient motor;
    // Mensagem destinada ao servico do motor
    dynamixel_workbench_msgs::JointCommand posicao;

public:
    PixhawkeMotor()
    {
        // Inicia a escuta do topico ja publicado de chegada da mensagem
        subPix = nh_.subscribe("/mavros/vfr_hud", 10, &PixhawkeMotor::escutarPixhawk, this);
        // Inicia a relacao de pwm/ang para cada motor
        pwm_ang_yaw   = (pwm_yaw_range[1] - pwm_yaw_range[0]) / (ang_yaw_range[1] - ang_yaw_range[0]);         // [PWM/DEGREES]
        pwm_ang_pitch = (pwm_pitch_range[1] - pwm_pitch_range[0]) / (ang_pitch_range[1] - ang_pitch_range[0]); // [PWM/DEGREES]
        // Pontos centrais dos dois servos
        yaw_mid_range   = (ang_yaw_range[1]   + ang_yaw_range[0])  /2; // [DEGREES]
        pitch_mid_range = (ang_pitch_range[1] + ang_pitch_range[0])/2; // [DEGREES]
        // Servico que envia valores ao motor
        motor = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");
    }

    int ExecutarClasse(int argc, char **argv)
    {
        calcularAngulosMotores();
        enviarAngulosMotores();
    }

private:
    void calcularAngulosMotores()
    {
      // Analisando diferenca de yaw
      delta_yaw = wrap180(yaw_atual, yaw_para_apontar);
      //ROS_INFO("delta yaw: %.2f", yaw_mid_range);
      ang_pan   = ((yaw_mid_range + delta_yaw) < ang_yaw_range[1]) ? yaw_mid_range + delta_yaw : ang_yaw_range[1]; // LImitando maximo
      ang_pan   = (ang_pan > ang_yaw_range[0]) ? ang_pan : ang_yaw_range[0]; // LImitando minimo

      // Analisando diferenca de pitch -> somente a mesma sobre o pwm para manter horizontal
      delta_pitch = ((ang_pitch_horizontal + pitch_para_apontar) < ang_pitch_range[1]) ? pitch_para_apontar : ang_pitch_range[1] - ang_pitch_horizontal;
      delta_pitch = ((ang_pitch_horizontal + delta_pitch       ) > ang_pitch_range[0]) ? delta_pitch        : ang_pitch_horizontal - ang_pitch_range[0];

      // Uma vez todos os angulos calculados, converter para valor de pwm para enviar aos motores
      pwm_pan  = pwm_yaw_range[0] + (ang_pan - ang_yaw_range[0])*pwm_ang_yaw;
      pwm_tilt = pwm_pitch_horizontal + delta_pitch*pwm_ang_pitch;
    }

    void enviarAngulosMotores()
    {
      ROS_INFO("Angulo PAN : %d", pwm_pan);
      ROS_INFO("Angulo TILT: %d", pwm_tilt);

      posicao.request.pan_pos  = (int)pwm_pan;
      posicao.request.tilt_pos = (int)pwm_tilt;
      posicao.request.unit     = "raw";

      posicao.response.pan_pos = (int)pwm_pan;
      posicao.response.tilt_pos = (int)pwm_tilt;

      motor.call(posicao);
    }

    float rad2deg(float rad)
    {
        return 180.0/3.1415926535 * rad;
    }

    // Mantem o angulo de diferenca entre 180 da forma mais logica no algoritmo
    float wrap180(float atual, float apontar)
    {
        float delta = atual - apontar;
        if(delta >  180.0) {delta -= 360.0;}
        if(delta < -180.0) {delta += 360.0;}

        return delta;
    }

    void escutarPixhawk(const mavros_msgs::VFR_HUDConstPtr& msg)
    {
        /// Relacoes obtidas da mensagem VFR_HUD vinda da placa
        pitch_para_apontar = rad2deg(msg->airspeed);     // [RAD] -> [DEGREES]
        yaw_atual          = msg->groundspeed;           // [DEGREES]
        // Ajusta chegada desse angulo que vai de -180 a +180
        yaw_para_apontar   = ((float)(msg->heading)*0.01 >= 0) ? (float)(msg->heading)*0.01 : yaw_atual; // [DEGREES]
//        yaw_para_apontar = msg->heading*0.01;
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

    ros::Rate rate(20);

    while(ros::ok())
    {
      pxm.ExecutarClasse(argc, argv);
      rate.sleep();
      ros::spinOnce();
    }
    return 0;
}
