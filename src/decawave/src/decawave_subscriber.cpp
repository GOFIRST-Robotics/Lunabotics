// Import Native C++ Libraries
#include <string>
#include <functional>
#include <memory>

// Import ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

std::string deca_topic = "decawave/Range";

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node{
    public:
    MinimalSubscriber() : Node("minimal_listener"), counter(0) {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        deca_topic, 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));//create subscriptions to topic and call topic_callback
    }
    private:
    void topic_callback(const nav_msgs::msg::Odometry & msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.distance << "'");
    }
    int counter;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;//declare subscription pointer
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  rclcpp::shutdown();
  return 0;
}