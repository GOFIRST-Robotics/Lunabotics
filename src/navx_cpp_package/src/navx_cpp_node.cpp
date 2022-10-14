// Native Libraries
#include <string>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "boost/array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"

// Custom Libraries
#include "ahrs/AHRS.h"
#include "AHRS.cpp"

// ROS Parameters
double frequency = 50.0;
bool euler_enable = false;
std::string device_path = "/dev/ttyACM0";
std::string frame_id = "imu_link";
int covar_samples = 100;

// Custom Data Structure
typedef struct {
  float ypr[3];
  float ang_vel[3];
  float accel[3];
} OrientationEntry;

// Global Variables
AHRS* imu;
int seq = 0;
std::vector<OrientationEntry> orientationHistory;
static const float DEG_TO_RAD = M_PI / 180.0F;
static const float GRAVITY = 9.81F; // m/s^2, if we actually go to the moon remember to change this
bool calculate_covariance(boost::array<double, 9> &orientation_mat, 
                          boost::array<double, 9> &ang_vel_mat, 
                          boost::array<double, 9> &accel_mat);

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("NavX", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Calculate orientation
    OrientationEntry curOrientation;
    curOrientation.ypr[0] = imu->GetRoll() * DEG_TO_RAD;
    curOrientation.ypr[1] = imu->GetPitch() * DEG_TO_RAD;
    curOrientation.ypr[2] = imu->GetYaw() * DEG_TO_RAD;
    curOrientation.ang_vel[0] = imu->GetRollRate() * DEG_TO_RAD;
    curOrientation.ang_vel[1] = imu->GetPitchRate() * DEG_TO_RAD;
    curOrientation.ang_vel[2] = imu->GetYawRate() * DEG_TO_RAD;
    curOrientation.accel[0] = imu->GetWorldLinearAccelX() * GRAVITY;
    curOrientation.accel[1] = imu->GetWorldLinearAccelY() * GRAVITY;
    curOrientation.accel[2] = imu->GetWorldLinearAccelZ() * GRAVITY;

    orientationHistory[seq % covar_samples] = curOrientation;

    auto message = std_msgs::msg::String();
    message.data = "NavX Data: " + std::to_string(curOrientation.ypr[0]);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Initialize the NavX IMU
  imu = new AHRS(device_path); // TODO make this work
  orientationHistory.resize(covar_samples);

  // Spin the node
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}

/**
 * Calculates the covariance matrices based on the orientation history and stores the results in the provided arrays
 * Returns true if the returned covariance is valid, otherwise false
 */
bool calculate_covariance(boost::array<double, 9> &orientation_mat, 
                          boost::array<double, 9> &ang_vel_mat, 
                          boost::array<double, 9> &accel_mat) {
  int count = std::min(seq-1, covar_samples);
  if (count < 2) {
    return false; // Did not calculate covariance
  }
  OrientationEntry avg = {};
  // Calculate averages
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < 3; j++) {
      avg.ypr[j] += orientationHistory[i].ypr[j];
      avg.ang_vel[j] += orientationHistory[i].ang_vel[j];
      avg.accel[j] += orientationHistory[i].accel[j];
    }
  }
  for (int j = 0; j < 3; j++) {
    avg.ypr[j] /= count;
    avg.ang_vel[j] /= count;
    avg.accel[j] /= count;
  }
  // Calculate covariance
  // See https://en.wikipedia.org/wiki/Covariance#Calculating_the_sample_covariance
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      int idx = 3*x + y;
      orientation_mat[idx] = 0;
      ang_vel_mat[idx] = 0;
      accel_mat[idx] = 0;
      // Average mean error difference
      for (int i = 0; i < count; i++) {
        orientation_mat[idx] += (orientationHistory[i].ypr[x] - avg.ypr[x]) * (orientationHistory[i].ypr[y] - avg.ypr[y]);
        ang_vel_mat[idx] += (orientationHistory[i].ang_vel[x] - avg.ang_vel[x]) * (orientationHistory[i].ang_vel[y] - avg.ang_vel[y]);
        accel_mat[idx] += (orientationHistory[i].accel[x] - avg.accel[x]) * (orientationHistory[i].accel[y] - avg.accel[y]);
      }
      // Normalize
      orientation_mat[idx] /= count - 1;
      ang_vel_mat[idx] /= count - 1;
      accel_mat[idx] /= count - 1;
    }
  }
  return true;
}