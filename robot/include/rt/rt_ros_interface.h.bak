/**
 * @file rt_ros_interface.h
 *
 */
#ifndef _RT_ROS_INTERFACE
#define _RT_ROS_INTERFACE

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"

class rc_control_settings {
  public:
    double     mode;
    double     p_des[2]; // (x, y) -1 ~ 1
    double     height_variation; // -1 ~ 1
    double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3]; // -1 ~ 1
    double     omega_des[3]; // -1 ~ 1
    double     variable[3];
};

class ROS_connect
{
  public:
  void init();

  private:
  void ROS_control_mode_Callback(const std_msgs::Int16::ConstPtr& mode);
  void ROS_twist_Callback(const geometry_msgs::Twist::ConstPtr& vel);
}


namespace RC_mode{
  constexpr int OFF = 0;
  constexpr int STAND_UP = 2;
  constexpr int QP_STAND = 3;
  constexpr int BACKFLIP_PRE = 4;
  constexpr int BACKFLIP = 5;
  constexpr int VISION = 6;
  constexpr int LOCOMOTION = 11;
  constexpr int RECOVERY_STAND = 12;

  // Experiment Mode
  constexpr int TWO_LEG_STANCE_PRE = 20;
  constexpr int TWO_LEG_STANCE = 21;
  constexpr int JP_TEST = 22;
};

#endif