#include <pthread.h>
#include <rt/rt_ros_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>

#include <iostream>

void ROS_connect::init()
{
  int a = 0;
  char * b =nullptr;
  ros::init(a, &b, "robodog_sub");

  ros::NodeHandle n;

  ROS_INFO("ROS node initialized");

  ros::AsyncSpinner spinner(1);

  spinner.start();

  rc_control.mode = 1;

  ros::Subscriber subt_rc_mode = n.subscribe("rc_mode", 100, ROS_controll_mode_Callback);

  ros::Subscriber subtwist = n.subscribe("cmd_vel", 100, ROS_twist_Callback);

  ros::waitForShutdown();
}


void ROS_connect::ROS_control_mode_Callback(const std_msgs::Int16::ConstPtr& mode)
{ 
  rc_control.mode = mode->data;
  std::cout<<"rc_mode:  "<<rc_control.mode<<std::endl;
}

void ROS_connect::ROS_twist_Callback(const geometry_msgs::Twist::ConstPtr& vel)
{ 
  rc_control.v_des[0] = vel->linear.x;
  rc_control.v_des[1] = vel->linear.y;
  rc_control.omega_des[2] = - vel->angular.z; //to align ROS linear basis
  rc_control.height_variation = vel->linear.z;
  rc_control.rpy_des[0] = vel->angular.x;
  rc_control.rpy_des[1] = vel->angular.y;
  rc_control.rpy_des[2] = - vel->angular.z; //to align ROS linear basis
}



/*
  int selected_mode = 0;
  switch(estop_switch) {
    case SWITCH_UP: // ESTOP
      selected_mode = RC_mode::OFF;
      break;
    case SWITCH_MIDDLE: // recover
      selected_mode = RC_mode::RECOVERY_STAND;
      break;
    case SWITCH_DOWN: // run 
      selected_mode = RC_mode::LOCOMOTION; // locomotion by default
      // stand mode
      if(left_select == SWITCH_UP && right_select == SWITCH_UP) {
        selected_mode = RC_mode::QP_STAND;
      }
      if(backflip_prep_edge_trigger.trigger(mode_selection_switch) 
          && mode_selection_switch == SWITCH_MIDDLE) {
        initial_mode_go_switch = mode_go_switch;
      }
      // Experiment mode (two leg stance, vision, ...)
      if(experiment_prep_edge_trigger.trigger(mode_selection_switch) 
          && mode_selection_switch == SWITCH_DOWN) {
        initial_mode_go_switch = mode_go_switch;
      }
      // backflip
      if(mode_selection_switch == SWITCH_MIDDLE) {
        selected_mode = RC_mode::BACKFLIP_PRE;
        if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) {
          selected_mode = RC_mode::BACKFLIP;
        } else if(mode_go_switch == SWITCH_UP) {
          initial_mode_go_switch = SWITCH_UP;
        }
      } // Experiment Mode
      else if(mode_selection_switch == SWITCH_DOWN){
        int mode_id = left_select * 3 + right_select;
        if(mode_id == 0){ // Two leg stance
          selected_mode = RC_mode::TWO_LEG_STANCE_PRE;
          if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) {
            selected_mode = RC_mode::TWO_LEG_STANCE;
          } else if(mode_go_switch == SWITCH_UP) {
            initial_mode_go_switch = SWITCH_UP;
          }
        }
        else if(mode_id == 1){ // Vision 
          selected_mode = RC_mode::VISION;
        }
      }
      // gait selection
      int mode_id = left_select * 3 + right_select;
      constexpr int gait_table[9] = {0, //stand
        0, // trot
        1, // bounding
        2, // pronking
        3, // gallop
        5, // trot run
        6, // walk};
        7, // walk2?
        8, // pace
  };
*/

void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}


float deadband(float command, float deadbandRegion, float minVal, float maxVal){
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}