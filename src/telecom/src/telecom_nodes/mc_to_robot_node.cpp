/*
 * mc_to_robot_node.cpp
 * Uses telecom to TX/RX ROS data from the Mission Control (MC) to the robot
 * VERSION: 0.0.0
 * Last changed: 2019-04-28
 * Authors: Michael Lucke <lucke096@umn.edu>
 * Maintainers: Michael Lucke <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

/* Wifi Transmissions
 * Robot Sends Jystick input
 */

// ROS Libs
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

// Native_Libs
#include <string>
#include <vector>

// Custom Library
#include "telecom/telecom.h"
#include "formatter_string/formatter.hpp"

// Subscribers (inputs)
//    joy_sub (sensor_msgs/Joy): joy
//      Subscribes to joystick data
//    state_sub (std_msgs/Bool): state_machine/isTeleopCtrl
//      Subscribes to turn on/off teleop control, keeps coms open/sending

// Publishers (outputs)
//    state_sub (std_msgs/Bool): state_machine/isTeleopCtrl
//      Receives 
//    pub_name1 (pub_name1_type): pub_name1_TOPIC_VALUE
//      pub_name1_desc

// Parameters (settings)
//    frequency (double): default = 50.0
//      Hertz, to check for msgs and send
//    dst_addr (string): default = "127.0.0.1"
//      The IPv4 address of the target device
//    dst_port (int): default = 5554
//      The port on target device to target
//    src_port (int): default = 5556
//      The port on host device to use
//    isTeleopCtrl (bool): default = true
//      Whether teleop control is enabled by default, initially

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
//ros::Publisher pub_name3_pub;

// ROS Topics
std::string joy_topic = "joy";
std::string state_topic = "state_machine/isTeleopCtrl";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void state_callback(const std_msgs::Bool::ConstPtr& msg);

// ROS Params
double frequency = 20.0;
std::string dst_addr = "192.168.1.19";
int dst_port = 5554;
int src_port = 5556;
bool isTeleopCtrl = true;

// Global_Vars
Telecom *com;
Formatter *fmt;
void update_fn();
std::string recv_msg;
double axes[6] = {0.0};
std::vector<IV> buttons_iv = {{0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {6,0},
  {7,0}, {8,0}, {9,0}, {10,0}, {11,0}};
std::vector<IV> pad_iv = {{0,0}};
std::vector<IV_float> axes_iv = {{0,0.0}, {1,0.0}, {2,0.0}, {3,0.0}};

#define ERR_CHECK() \
  do { if (com->status() != 0){ \
    fprintf(stdout, "Error: %s\n", com->verboseStatus().c_str()); \
    exit(com->status()); \
  } } while(0)

// Formatters
val_fmt js_axes_msg_fmt = {
  "js_axes_msg_fmt",
  '!',
  6, // # bytes
  0, // Min val
  200000, // Max val
  100000, // Offset
  100000  // Scale
};

// byte_msg_fmt
val_fmt button_msg_fmt = {
  "button_msg_fmt",
  '@',
  1,
  0,
  255,
  0,
  1
};

// pad_msg_fmt
val_fmt pad_msg_fmt = {
  "pad_msg_fmt",
  '#',
  1,
  0,
  255,
  0,
  1
};

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "mc_to_robot_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Params
  pnh->getParam("frequency", frequency);
  pnh->getParam("dst_addr", dst_addr);
  pnh->getParam("dst_port", dst_port);
  pnh->getParam("src_port", src_port);
  pnh->getParam("isTeleopCtrl", isTeleopCtrl);

  // Init variables
  fmt = new Formatter({js_axes_msg_fmt, button_msg_fmt, pad_msg_fmt});
  com = new Telecom(dst_addr, dst_port, src_port);
  com->setFailureAction(false);
  com->setBlockingTime(0,0);
  ERR_CHECK();

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  ros::Subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);

  // Publishers

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&){
  update_fn();
}

void update_fn(){
  com->update();
  ERR_CHECK();

  // Read from com for msg
  if(com->recvAvail()){
    recv_msg = com->recv();
    // For testing
    if(!recv_msg.empty())
      printf("Received message: %s\n", recv_msg.c_str());
  }
  // Safety/bug?
  while(com->isComClosed()){
    printf("Rebooting connection\n");
    com->reboot();
    if(!com->isComClosed()){
      printf("Reconnected to %s:%i\n", dst_addr.c_str(), dst_port);
    }
  }

  // Process recv_msg
  
  // Send joystick
  fmt->addFloat("js_axes_msg_fmt", axes_iv);
  fmt->add("button_msg_fmt", buttons_iv);
  fmt->add("pad_msg_fmt", pad_iv);
  com->send(fmt->emit());
}

/* Transmition format
 * A bit field byte of button states: 0-7 are
 * X (TRNS CONV), A (DIGR), B (), Y ()
 * LB (DOOR UP), RB (DOOR DN), LT (HOLD CONV OUT), RT (HOLD CONV IN)
 * A value of pad states: 0-4 are
 * 0 (NONE), LF (LF UP), RT (RT UP), UP (BOTH UP), DN (BOTH DN)
 */
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  // Process buttons
  buttons_iv[0].v = 0;
  for(int i = 0; i < 12; ++i){
    buttons_iv[i].v = msg->buttons[i];
  }
  // Process axes
  for(int i = 0; i < 6; ++i){
    axes[i] = msg->axes[i];
    if(i < 4){
      axes_iv[i].v = axes[i];
    }
  }
  // Process pad
        if(axes[5] >  0.1){
    pad_iv[0].v = 3;
  }else if(axes[5] < -0.1){
    pad_iv[0].v = 4;
  }else if(axes[4] >  0.1){
    pad_iv[0].v = 1;
  }else if(axes[4] < -0.1){
    pad_iv[0].v = 2;
  }else{
    pad_iv[0].v = 0;
  }
  update_fn();
}

void state_callback(const std_msgs::Bool::ConstPtr& msg){
  isTeleopCtrl = msg->data;
}
