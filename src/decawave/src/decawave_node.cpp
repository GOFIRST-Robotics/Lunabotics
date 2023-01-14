/*
 * decawave_node.cpp
 * ROS2 interface to Decawave class
 * VERSION: 1.0.1
 * Last changed: 2022-12-14
 * Authors: Eric Patton <patto164@umn.edu>, Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainer: Eric Patton <patto164@umn.edu>
 * MIT License
 * Copyright (c) 2022 GOFIRST-Robotics
 */

// Import ROS Libraries
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Import Native Libraries
#include <string>

// Import Custom Libraries
#include "decawave/decawave.h"
#include "decawave/Range.h"

// ROS Publisher Topic
std::string deca_topic = "decawave/Range";

// ROS@ Parameters // TODO: Not setup as parameters yet
double frequency = 50.0;
int port_num = 0;
// for(int i = 0; i < argc; ++i){
// int port_num = atoi(argv[1]);
// }

// Global Variables
Decawave *decawave_sensor;
decawave_coordinate decaPos;
// Decawave piTag(1);
// decawave_coordinate tagPos;

using namespace std::chrono_literals;

class Decawave_Publisher : public rclcpp::Node{
  public: Decawave_Publisher() : Node("decawave_node"){
    deca_topic = deca_topic + std::to_string(port_num);
    decawave_pub = this->create_publisher<nav_msgs::msg::Odometry>(deca_topic, 1);
    timer = this->create_wall_timer(500ms, std::bind(&Decawave_Publisher::timer_callback, this));
  }
  private:
    //publish info from sensor
    void timer_callback(/*const ros::TimerEvent&*/){
      // ROS_INFO("Publishing...");
      RCLCPP_INFO(this->get_logger(), "Publishing...");
      
      auto deca_msg = nav_msgs::msg::Odometry();
      std::string frame_id = std::to_string(port_num);
      // update decawave data
      decawave_sensor->updateSamples();
      // get tag position from the decawave
      decaPos = decawave_sensor->getPos();
      deca_msg.distance = decaPos.x;
      deca_msg.estimated_variance = 0.1;

      deca_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      deca_msg.header.frame_id = "decawave" + frame_id;
      deca_msg.child_frame_id = "decawave2_link";
      
      decawave_pub->publish(deca_msg);
      deca_msg.distance = decaPos.y;
      deca_msg.estimated_variance = 0.1;
      
      deca_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      deca_msg.header.frame_id = "decawave" + frame_id;
      deca_msg.child_frame_id = "decawave3_link";
      decawave_pub->publish(deca_msg);
      
      deca_msg.pose.pose.position.x;//y, z, maybe not the second .pose
      
      /*
      decawave::Range msg;
      msg.distance = tagPos.x;
      msg.estimated_variance = 0.10;
      // fill out message header
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
      msg.child_frame_id = "decawave2_link"; //change to correct part
      //
      gps_pub->publish(msg);
      //
      msg.distance = tagPos.y;
      msg.estimated_variance = 0.10;
      // fill out message header
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
      msg.child_frame_id = "decawave3_link"; //change to correct part
      //
      gps_pub->publish(msg);*/
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr decawave_pub;
};

// Main method for the node
int main(int argc, char** argv){
  // Init ROS
  //ros::init(argc, argv);
  // nh = new ros::NodeHandle("");
  // pnh = new ros::NodeHandle("~");
  //
  // Params
  // pnh->param<double>("frequency", frequency);
  // nh->param<int>("port_num",port_num);
  // port_num = nh->getParam("port_num",port_num);
  // int port_num_test = ros::param::get("~port_num",port_num);
  //ros::param::get("~port_num",port_num);
  // debug:
  // std::string s = std::to_string(port_num);
  // char const *pchar = s.c_str();  //use char const* as target type
  // ROS_INFO(pchar);
  //printf("%i\n",port_num);
  // printf("%i\n",port_num_test);
  //nh->param<param_name3_type>(param_name3_path, param_name3, param_name3_default;
  //ros::init(argc, argv, "decawave_node" + std::to_string(port_num));
  //auto node = rclcpp::Node::make_shared("decawave_node");
  //nh = new ros::NodeHandle("");
  //pnh = new ros::NodeHandle("~");
  // Initialize
  //piTag = new Decawave(port_num);
  // get initial decwave data
  // ROS_INFO("Dw created");
  /*for(int i = 0; i < 10; ++i){
    // ROS_INFO("Updating Samples");
    piTag->updateSamples();
    // ROS_INFO("Updated Sample");//make an edit
  }*/
  //
  // Subscribers
  //ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  //auto update_timer = node->create_subscription<cpp
  //
  // Publishers
  //deca_topic = deca_topic + std::to_string(port_num);
  //gps_pub = nh->advertise<decawave::Range>(deca_topic, 10);
  // Spin
  //ros::spin();

  // Initialize ROS
  rclcpp::init(argc, argv);

  // Initialize the Decawave sensor
  decawave_sensor = new Decawave(port_num);

  // Spin the node
  rclcpp::spin(std::make_shared<Decawave_Publisher>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}