// Native Libraries
#include <memory>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber()
    : Node("subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Imu & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieving NavX Orientation: (%f, %f, %f)", msg.orientation.x, msg.orientation.y, msg.orientation.z);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();

  return 0;
}