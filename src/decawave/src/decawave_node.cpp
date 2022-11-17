/*
 * decawave_node.cpp
 * ROS interface to Decawave class
 * VERSION: 2.0
 * Last changed: 2022-11-16
 * Authors: Eric Patton <patto164@umn.edu>, Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainers: Eric Patton <patto164@umn.edu>
 * MIT License
 * Copyright (c) 2022 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Native_Libs
#include <string>

// Custom_Libs
#include "decawave/decawave.h"
#include "decawave/Range.h"

// Subscribers (inputs)
//    update_timer (Timer)
//      Update loop for reading / querying Decawave
//    sub_name1 (sub_name1_type): sub_name1_TOPIC_VALUE
//      sub_name1_desc

// Publishers (outputs)
//    gps_pub (nav_msgs/Odometry): "odometry/gps"
//      A simulated local position as if GPS UTM

// Parameters (settings)
//    frequency (double): default=50.0
//      The update frequency of the update loop
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher gps_pub;

// ROS Topics
std::string gps_topic = "decawave/Range";//"odometry/gps";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency = 50.0;
int port_num = 0;
// for(int i = 0; i < argc; ++i){
// int port_num = atoi(argv[1]);
// }

// Global_Vars
Decawave *piTag;
decawave_coordinate tagPos;
// Decawave piTag(1);
// decawave_coordinate tagPos;

//start things
int main(int argc, char** argv){
  // Init ROS
  // ros::init(argc, argv, "decawave_node");
  // nh = new ros::NodeHandle("");
  // pnh = new ros::NodeHandle("~");

  // Params
  // pnh->param<double>("frequency", frequency);
  // nh->param<int>("port_num",port_num);
  // port_num = nh->getParam("port_num",port_num);
  // int port_num_test = ros::param::get("~port_num",port_num);
  ros::param::get("~port_num",port_num);
  // debug:
  // std::string s = std::to_string(port_num);
  // char const *pchar = s.c_str();  //use char const* as target type
  // ROS_INFO(pchar);
  printf("%i\n",port_num);
  // printf("%i\n",port_num_test);
  //nh->param<param_name3_type>(param_name3_path, param_name3, param_name3_default;
  ros::init(argc, argv, "decawave_node" + std::to_string(port_num));
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");
  // Initialize
  piTag = new Decawave(port_num);
  // get initial decwave data
  // ROS_INFO("Dw created");
  for(int i = 0; i < 10; ++i){
    // ROS_INFO("Updating Samples");
    piTag->updateSamples();
    // ROS_INFO("Updated Sample");
  }

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  gps_topic = gps_topic + std::to_string(port_num);
  gps_pub = nh->advertise<decawave::Range>(gps_topic, 10);

  // Spin
  ros::spin();
}

//publish info from sensor
void update_callback(const ros::TimerEvent&){
  // ROS_INFO("Publishing..");
  decawave::Range msg;

  // update decawave data
  piTag->updateSamples();

  // get tag position from the decawave
  tagPos = piTag->getPos();

  ros::param::get("~port_num",port_num);
  std::string frame_id = std::to_string(port_num);

  msg.distance = tagPos.x;
  msg.estimated_variance = 0.10;
  // fill out message header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
  msg.child_frame_id = "decawave2_link"; //change to correct part

  gps_pub.publish(msg);

  msg.distance = tagPos.y;
  msg.estimated_variance = 0.10;
  // fill out message header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
  msg.child_frame_id = "decawave3_link"; //change to correct part

  gps_pub.publish(msg);
}