#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include <Arduino.h>
#include <TimerThree.h>
/* ros_communication msg */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

/* Pin information */
#define E1_CHA 2
#define E1_CHB 3
#define M1_DIR A2
#define M1_PWM A3

#define E2_CHA 18
#define E2_CHB 19
#define M2_DIR A6
#define M2_PWM A7

/* Mobile and Wheel information */
#define EncoderCountsPerWheel 1976  // per 1 cycle
#define wheelradius 0.076           // units : m
#define wheelbase 1000              // units : m/s

#define M_PI 3.14159265358979323846

extern float DistancePerCount;

/* ROS Callback */
void CmdVelCallback(const geometry_msgs::Twist& msg);
    
/* Motor Class */
class Motor
{
  public:
    Motor(int E_CHA, int E_CHB, int M_DIR, int M_PWM, float K_P, float K_I, float K_D);
    ~Motor();
    void EnCHA_ISR();
    void EnCHB_ISR();
    void SpeedControl(float ref_speed);
    void EncoderCounter();
    void SerialRead();

    int enPos_;
    float m_speed_;
    float ref_speed_;
    
    int E_CHA_, E_CHB_, M_DIR_, M_PWM_;
    float K_P_, K_D_, K_I_;

  private:
    float err_speed;
    float err_speed_k_1;
    float err_speed_sum;
    float err_speed_dot;
    float prev;
};

/* control function */

void UpdateVel(geometry_msgs::Twist cmd_vel);
