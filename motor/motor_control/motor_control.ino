#include "motor_control.h"

/* Declare Variable */
#define motor_control_freqeuncy 10
#define odom_cal_freqeuncy 100

double Time[2] = {0, };
geometry_msgs::Pose2D odom;

/* Motor Class Initialization */
Motor Motor1(E1_CHA, E1_CHB, M1_DIR, M1_PWM, 20.0, 0.0, 0.0);
Motor Motor2(E2_CHA, E2_CHB, M2_DIR, M2_PWM, 20.0, 0.0, 0.0);

/* Ros */
ros::NodeHandle nh;

ros::Publisher pub_odom_delta("/mobile/odom_delta", &odom);
ros::Subscriber<geometry_msgs::Twist> sub_cmd("/mobile/cmd_vel", CmdVelCallback);

void setup()
{ 
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  attachInterrupt(E1_CHA, callback1, CHANGE);
  attachInterrupt(E1_CHB, callback2, CHANGE);
  attachInterrupt(E2_CHA, callback3, CHANGE);
  attachInterrupt(E2_CHB, callback4, CHANGE);

  odom.x = 0.0;
  odom.y = 0.0;
  odom.theta = 0.0;
  
  nh.initNode();
  nh.advertise(pub_odom_delta);
  nh.subscribe(sub_cmd);
}

void loop()
{
  double t = millis();

  if((t - Time[0]) >= motor_control_freqeuncy)
  {
    motor_ISR((t - Time[0])/1000);
    Time[0] = t;
  }
  else if((t - Time[1]) >= odom_cal_freqeuncy)
  {
    Calc_Odom((t - Time[1])/1000);
    Time[1] = t;
  }

  nh.spinOnce();
}

/* Motor Velocity Update Function */
void CmdVelCallback(const geometry_msgs::Twist& msg)
{
  UpdateVel(msg);
}

void UpdateVel(geometry_msgs::Twist cmd_vel)
{
  double goal_velocity_x = cmd_vel.linear.x;
  double goal_velocity_z = cmd_vel.angular.z;

  Motor1.ref_speed_ = (goal_velocity_x - goal_velocity_z * wheelbase / 2);
  Motor2.ref_speed_ = (goal_velocity_x + goal_velocity_z * wheelbase / 2);  
}

/* Motor Control Function */
void motor_ISR(double t)
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

/* Calculate Odometry Function */
void Calc_Odom(double t)
{
  double velL = Motor1.m_speed_;
  double velR = Motor2.m_speed_;

  double linear_vel = (velL + velR) / 2;
  double angular_vel = (velR - velL) / wheelbase;

  double distance_delta = linear_vel * t;
  double angular_delta = angular_vel * t;

  odom.x = distance_delta * cos(odom.theta + angular_delta / 2);
  odom.y = distance_delta * sin(odom.theta + angular_delta / 2);
  odom.theta = angular_delta;

  pub_odom_delta.publish(&odom);
}

/* Encoder Update Function */
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
