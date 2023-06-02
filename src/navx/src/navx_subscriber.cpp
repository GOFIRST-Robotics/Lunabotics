// Import Native C++ Libraries
#include <memory>

// Import ROS 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class NavxSubscriber : public rclcpp::Node
{
public:
  NavxSubscriber() : Node("NavXSubscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 1, std::bind(&NavxSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Orientation: (%f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z);
    RCLCPP_INFO(this->get_logger(), "Velocity: (%f, %f, %f)", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    RCLCPP_INFO(this->get_logger(), "Acceleration: (%f, %f, %f)", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<NavxSubscriber>());
  rclcpp::shutdown();

  return 0;
}