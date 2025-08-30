#include <memory>
#include <string>
#include <limits> // Required for std::numeric_limits

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // Header for LaserScan

class LaserDataNode : public rclcpp::Node
{
public:
  LaserDataNode()
  : Node("LaserDataNode")
  {
    // Create a new subscription for the /scan topic
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LaserDataNode::scan_callback, this, std::placeholders::_1));

    // Create a timer to periodically print the minimum distance
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&LaserDataNode::timer_callback, this));

    // Initialize with NaN
    min_distance_ = std::numeric_limits<float>::quiet_NaN(); 
  }
private:
  // Callback function for laser scan data
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_dist = std::numeric_limits<float>::max();
    bool found_valid_reading = false;

    // Iterate through the ranges to find the minimum distance
    for (const float &range : msg->ranges) {
      if (std::isinf(range) || std::isnan(range) || range <= msg->range_min || range >= msg->range_max) {
        continue; // Skip invalid readings
      }
      if (range < min_dist) {
        min_dist = range;
        found_valid_reading = true;
      }
    }

    if (found_valid_reading) {
      min_distance_ = min_dist; // Store the minimum distance
    } else {
      min_distance_ = std::numeric_limits<float>::quiet_NaN(); // Set to NaN if no valid readings
    }
  }

  // Timer callback to periodically print the minimum distance
  void timer_callback()
  {
    if (!std::isnan(min_distance_)) {
      RCLCPP_INFO(this->get_logger(), "Minimum distance to obstacle: %f meters", min_distance_);
    } else {
      RCLCPP_INFO(this->get_logger(), "No valid LiDAR readings detected.");
    }
  }

  // Current minimum distance from the LiDAR
  float min_distance_;

  // Subscriptions and timer
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserDataNode>());
  rclcpp::shutdown();
  return 0;
}
