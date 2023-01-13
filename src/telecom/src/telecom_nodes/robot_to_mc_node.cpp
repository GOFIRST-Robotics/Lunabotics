/*
 * robot_to_mc_node.cpp
 * Uses telecom to TX/RX ROS data from robot to the Mission Control (MC)
 * Does not transmit video
 * VERSION: 0.0.0
 * Last changed: 2019-04-28
 * Authors: Michael Lucke <lucke096@umn.edu>
 * Maintainers: Michael Lucke <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

/* Wifi Transmissions
 * Robot Sends Joystick input
 */

// ROS Libs
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Joy.h>

// Native_Libs
#include <string>
#include <vector>

// Custom Library
#include "telecom/telecom.h"
#include "formatter_string/formatter.hpp"

// Subscribers (inputs)
//    some_sensor_sub (sub_name2_type): sub_name2_TOPIC_VALUE
//      subscribes to some sensor topics with data we want to transmit if we find one

// Publishers (outputs)
//    joy_pub (sensor_msgs/Joy): joy
//      Publishes received joystick data

// Parameters (settings)
//    frequency (double): default = 50.0
//      Hertz, to check for msgs and send
//    dst_addr (string): default = "127.0.0.1"
//      The IPv4 address of the target device
//    dst_port (int): default = 5556
//      The port on target device to target
//    src_port (int): default = 5554
//      The port on host device to use
//    isTeleopCtrl (bool): default = true
//      Whether teleop control is enabled by default, initially


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher joy_pub;

// ROS Topics
std::string joy_topic = "joy";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
double frequency = 100.0;
std::string dst_addr = "192.168.1.10";
int dst_port = 5556;
int src_port = 5554;
bool isTeleopCtrl = true; 

// Global_Vars
Telecom *com;
Formatter *fmt;
std::string recv_msg;
void joy_pub_fn();

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
  ros::init(argc, argv, "robot_to_mc_node");
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
  
  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  //ros::Subscriber sub_name2_sub = nh->subscribe(sub_name2_topic, sub_name2_BUFLEN, sub_name2_callback);

  // Publishers
  joy_pub = nh->advertise<sensor_msgs::Joy>(joy_topic, 1);

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&){
  com->update();
  ERR_CHECK();

  // Read from com for msg
  if(com->recvAvail()){
    recv_msg = com->recv();

    // Safety/bug?
    while(com->isComClosed()){
      printf("Rebooting connection\n");
      com->reboot();
    }

    // For testing
    //if(!recv_msg.empty()) printf("Received message: %s\n", recv_msg.c_str());

    // Process recv_msg
    joy_pub_fn();

    // other pub
  }
}

void joy_pub_fn(){
  sensor_msgs::Joy joy_msg;
  joy_msg.axes.resize(6, 0.0);
  joy_msg.buttons.resize(12, 0.0);
  for(auto iv : fmt->parseFloat(recv_msg, "js_axes_msg_fmt")){
    joy_msg.axes[iv.i] = iv.v;
  }
  for(auto iv : fmt->parse(recv_msg, "button_msg_fmt")){
    joy_msg.buttons[iv.i] = iv.v;
  }
  for(auto iv : fmt->parse(recv_msg, "pad_msg_fmt")){
    if(iv.v == 1){
      joy_msg.axes[4] = 1.0;
    }else if(iv.v == 2){
      joy_msg.axes[4] = -1.0;
    }else if(iv.v == 3){
      joy_msg.axes[5] = 1.0;
    }else if(iv.v == 4){
      joy_msg.axes[5] = -1.0;
    }
  }
  joy_msg.header.stamp = ros::Time::now();
  joy_pub.publish(joy_msg);
}

/*
void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg){
//:BEGIN sub_name1_callback

//:END
}*/
