#pragma once

#include "teb_g2o_types.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robot {

class ControlCore {
public:
  ControlCore(const rclcpp::Logger& logger);
  ~ControlCore();

  // external setters (path, odom)
  void setGlobalPath(const nav_msgs::msg::Path::SharedPtr &path);
  void setOdometry(const nav_msgs::msg::Odometry::SharedPtr &odom);
  void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &map);
  geometry_msgs::msg::Twist computeVelocityCommand();

  // data availability checks
  bool hasPath() const { return current_path_ != nullptr && !current_path_->poses.empty(); }
  bool hasOdometry() const { return current_odom_ != nullptr; }
  bool hasOccupancyGrid() const { return current_map_ != nullptr; }
  bool isGoalReached() const;

private:
  rclcpp::Logger logger_;

  // helpers
  double extractYaw(const geometry_msgs::msg::Quaternion& q);

// internal data
  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  std::unique_ptr<g2o::SparseOptimizer> optimizer_;
  std::vector<Eigen::Vector3d> teb_poses_; // x,y,theta for trajectory
  bool optimized_{false};

  // parameters (tunable)
  double max_velocity_{1.0};      // m/s
  double max_angular_velocity_{1.0}; // rad/s
  double goal_tolerance_{0.3};    // m
  double obstacle_inflation_{0.3}; // m
  double path_sample_distance_{0.3}; // meters between initial samples
  
  // optimization helpers
  void clearGraph();
  void initializeFromPath();
  void optimizeTEB();
  bool getClosestObstacle(const Eigen::Vector2d& p, Eigen::Vector2d& obs_point, double max_search);

};

} // namespace robot