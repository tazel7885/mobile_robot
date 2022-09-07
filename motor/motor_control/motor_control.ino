#include "motor_control.h"
#include <TimerThree.h>

/* Motor Class Initialization */
Motor Motor1(E1_CHA, E1_CHB, M1_DIR, M1_PWM, 20.0, 0.0, 0.0);
Motor Motor2(E2_CHA, E2_CHB, M2_DIR, M2_PWM, 20.0, 0.0, 0.0);

/* Ros Subscriber */
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_cmd("/mobile/cmd_vel", CmdVelCallback);

void setup()
{ 
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
    
  Timer3.initialize(10000);   // units : 10000microsec = 10ms
  Timer3.attachInterrupt(timer1_ISR);
  
  attachInterrupt(E1_CHA, callback1, CHANGE);
  attachInterrupt(E1_CHB, callback2, CHANGE);
  attachInterrupt(E2_CHA, callback3, CHANGE);
  attachInterrupt(E2_CHB, callback4, CHANGE);
  
  nh.initNode();
  nh.subscribe(sub_cmd);
}

void loop()
{
  nh.spinOnce();
  delayMicroseconds(1);
}

void CmdVelCallback(const geometry_msgs::Twist& msg)
{
  UpdateVel(msg);
}

void callback1(){
  Motor1.EnCHA_ISR();
}
void callback2(){
  Motor1.EnCHB_ISR();
}

void callback3(){
  Motor2.EnCHA_ISR();
}
void callback4(){
  Motor2.EnCHB_ISR();
}

void timer1_ISR()
{
  Motor1.m_speed_ = Motor1.enPos_ * DistancePerCount / 0.01;
  Motor1.SpeedControl(Motor1.ref_speed_);
  
  Motor2.m_speed_ = Motor2.enPos_ * DistancePerCount / 0.01;
  Motor2.SpeedControl(Motor2.ref_speed_);

  Motor1.enPos_ = Motor2.enPos_ = 0;

  /* Debug */
  // Serial.println("--------------------------------------");
  // Motor1.SerialRead();
  // Motor1.EncoderCounter();
}

void UpdateVel(geometry_msgs::Twist cmd_vel)
{
  double goal_velocity_x = cmd_vel.linear.x;
  double goal_velocity_z = cmd_vel.angular.z;

  Motor1.ref_speed_ = (goal_velocity_x - goal_velocity_z * wheelbase / 2);
  Motor2.ref_speed_ = (goal_velocity_x + goal_velocity_z * wheelbase / 2);  
}
