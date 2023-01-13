/*
 * mc_to_robot_node.cpp
 * Uses telecom to TX/RX ROS data from the Mission Control (MC) to the robot
 * VERSION: 0.0.2
 * Last changed: 2019-04-28
 * Authors: Michael Lucke <lucke096@umn.edu>
 * Maintainers: Michael Lucke <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

/* Wifi Transmissions
 * Mission Control Sends Joystick input
 */

// ROS Libs
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

// Native_Libs
#include <string>
#include <vector>

// Custom Library
#include "telecom/telecom.h"
#include "telecom/telecom.cpp"
#include "formatter_string/formatter.hpp"

// Subscribers (inputs)
//    joy_sub (sensor_msgs/Joy): joy
//      Subscribes to joystick data

// Publishers (outputs)
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

// ROS Topics
std::string joy_topic = "joy";

// Settings
double frequency = 20.0;
std::string dst_address = "192.168.1.19";
int dst_port_num = 5554;
int src_port_num = 5556;

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
      printf("Reconnected to %s:%i\n", dst_address.c_str(), dst_port_num);    
    }
  }

  // Process recv_msg
  
  // Send joystick
  fmt->addFloat("js_axes_msg_fmt", axes_iv);
  fmt->add("button_msg_fmt", buttons_iv);
  fmt->add("pad_msg_fmt", pad_iv);
  com->send(fmt->emit());
}

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublishersAndSubscribers : public rclcpp::Node
{
  public:
  PublishersAndSubscribers()
  : Node("publishers_and_subscribers")
  {
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, 1, std::bind(&PublishersAndSubscribers::joy_callback, this, _1));
    timer = this->create_wall_timer(500ms, std::bind(&PublishersAndSubscribers::update_callback, this));
  }

private:
  void update_callback(){
    update_fn();
  }

    /* Transmition format
  * A bit field byte of button states: 0-7 are
  * X (TRNS CONV), A (DIGR), B (), Y ()
  * LB (DOOR UP), RB (DOOR DN), LT (HOLD CONV OUT), RT (HOLD CONV IN)
  * A value of pad states: 0-4 are
  * 0 (NONE), LF (LF UP), RT (RT UP), UP (BOTH UP), DN (BOTH DN)
  */
  void joy_callback(const sensor_msgs::msg::Joy & msg){
    // Process buttons
    buttons_iv[0].v = 0;
    for(int i = 0; i < 12; ++i){
      buttons_iv[i].v = msg.buttons[i];
    }
    // Process axes
    for(int i = 0; i < 6; ++i){
      axes[i] = msg.axes[i];
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
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
};

int main(int argc, char** argv){
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Initialize variables
  fmt = new Formatter({js_axes_msg_fmt, button_msg_fmt, pad_msg_fmt});
  com = new Telecom(dst_address, dst_port_num, src_port_num);
  com->setFailureAction(false);
  com->setBlockingTime(0,0);
  ERR_CHECK();

  // Spin the node
  rclcpp::spin(std::make_shared<PublishersAndSubscribers>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}