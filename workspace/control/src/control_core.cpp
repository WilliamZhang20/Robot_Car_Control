#include "control_core.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <utility>
#include <geometry_msgs/msg/twist.hpp>

// g2o solver helpers
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace robot;

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {
  // initialize optimizer once and reuse it
  using namespace g2o;
  optimizer_ = std::make_unique<SparseOptimizer>();
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > BlockSolverX;
  typedef g2o::LinearSolverCSparse<BlockSolverX::PoseMatrixType> LinearSolverType;
  auto linearSolver = std::make_unique<LinearSolverType>();
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer_->setAlgorithm(solver);
  optimizer_->setVerbose(false);
}

ControlCore::~ControlCore() {
  clearGraph();
  optimizer_.reset();
}

void ControlCore::setGlobalPath(const nav_msgs::msg::Path::SharedPtr &path) {
  current_path_ = path;
  // Clear existing trajectory to force reinitialisation with new path - RESPOND TO GOAL CHANGES!
  teb_poses_.clear();
  optimized_ = false;
}

void ControlCore::setOdometry(const nav_msgs::msg::Odometry::SharedPtr &odom) {
  current_odom_ = odom;
}

void ControlCore::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &map) {
  current_map_ = map;
}

// small helpers
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void ControlCore::clearGraph() {
  // clear vertices and edges but keep the optimizer and sampled TEB
  if (optimizer_) {
    optimizer_->clear();
    RCLCPP_DEBUG(logger_, "clearGraph: cleared optimizer graph");
  } else {
    RCLCPP_WARN(logger_, "clearGraph: optimizer is null - this should not happen");
  }
  // don't clear teb_poses_ here - keep samples between runs
  optimized_ = false;
}

void ControlCore::initializeFromPath() {
  if (!current_path_ || current_path_->poses.empty()) return;

  teb_poses_.clear();
  // use robot odom as first pose if available
  if (current_odom_) {
    Eigen::Vector3d start;
    start.x() = current_odom_->pose.pose.position.x;
    start.y() = current_odom_->pose.pose.position.y;
    start.z() = extractYaw(current_odom_->pose.pose.orientation);
    teb_poses_.push_back(start);
  }

  // sample along path, but skip poses that are too close to obstacles
  Eigen::Vector3d last = teb_poses_.empty() ? Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0) : teb_poses_.back();
  double acc_dist = 0.0;
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double x = current_path_->poses[i].pose.position.x;
    double y = current_path_->poses[i].pose.position.y;
    
    // Check if this pose is too close to obstacles
    Eigen::Vector2d pose_pos(x, y);
    Eigen::Vector2d nearest_obs;
    bool too_close_to_obstacle = false;
    if (getClosestObstacle(pose_pos, nearest_obs, 1.0)) {
      double obs_dist = (pose_pos - nearest_obs).norm();
      if (obs_dist < obstacle_inflation_ + 0.1) {
        too_close_to_obstacle = true;
        RCLCPP_DEBUG(logger_, "Skipping path pose at (%f,%f) - too close to obstacle (dist=%f)", x, y, obs_dist);
      }
    }
    
    // Skip poses that are in collision
    if (too_close_to_obstacle) continue;
    
    double yaw = 0.0; // fallback
    if (i+1 < current_path_->poses.size()) {
      double nx = current_path_->poses[i+1].pose.position.x;
      double ny = current_path_->poses[i+1].pose.position.y;
      yaw = std::atan2(ny - y, nx - x);
    } else if (!std::isnan(last.x())) {
      yaw = last.z();
    }

    if (teb_poses_.empty()) {
      teb_poses_.push_back(Eigen::Vector3d(x,y,yaw));
      last = teb_poses_.back();
    } else {
      double d = std::hypot(x - last.x(), y - last.y());
      acc_dist += d;
      if (acc_dist >= path_sample_distance_) {
        teb_poses_.push_back(Eigen::Vector3d(x,y,yaw));
        acc_dist = 0.0;
        last = teb_poses_.back();
      }
    }
    if (teb_poses_.size() >= 40) break; // cap samples
  }

  if (teb_poses_.size() == 1 && current_path_->poses.size() > 1) {
    // ensure at least two points - use goal if it's not in collision
    double goal_x = current_path_->poses.back().pose.position.x;
    double goal_y = current_path_->poses.back().pose.position.y;
    Eigen::Vector2d goal_pos(goal_x, goal_y);
    Eigen::Vector2d nearest_obs;
    bool goal_safe = true;
    if (getClosestObstacle(goal_pos, nearest_obs, 1.0)) {
      double obs_dist = (goal_pos - nearest_obs).norm();
      if (obs_dist < obstacle_inflation_ + 0.1) {
        goal_safe = false;
      }
    }
    
    if (goal_safe) {
      teb_poses_.push_back(Eigen::Vector3d(goal_x, goal_y, 0.0));
    }
  }

  if (!teb_poses_.empty()) {
    RCLCPP_INFO(logger_, "initializeFromPath: sampled %zu TEB poses first=(%f,%f) last=(%f,%f)", teb_poses_.size(), teb_poses_.front().x(), teb_poses_.front().y(), teb_poses_.back().x(), teb_poses_.back().y());
  } else {
    RCLCPP_WARN(logger_, "initializeFromPath: no TEB poses created from path");
  }
}

bool ControlCore::getClosestObstacle(const Eigen::Vector2d& p, Eigen::Vector2d& obs_point, double max_search) {
  if (!current_map_) return false;
  const auto &map = *current_map_;
  double res = map.info.resolution;
  double min_dist = std::numeric_limits<double>::infinity();
  bool found = false;

  // map origin
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;
  int width = map.info.width;
  int height = map.info.height;

  // naive scan - fine for small maps; can be optimized later
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = x + y * width;
      int occ = static_cast<int>(map.data[idx]);
      if (occ < 0) continue; // unknown
      if (occ >= 50) {
        double wx = ox + (x + 0.5) * res;
        double wy = oy + (y + 0.5) * res;
        double dist = std::hypot(wx - p.x(), wy - p.y());
        if (dist < min_dist && dist <= max_search) {
          min_dist = dist;
          obs_point = Eigen::Vector2d(wx, wy);
          found = true;
        }
      }
    }
  }
  return found;
}

void ControlCore::optimizeTEB() {
  if (teb_poses_.empty()) initializeFromPath();
  RCLCPP_INFO(logger_, "optimizeTEB: starting with teb_poses=%zu", teb_poses_.size());
  if (teb_poses_.size() < 2) return;

  using namespace g2o;
  clearGraph();
  RCLCPP_INFO(logger_, "optimizeTEB: after clearGraph teb_poses=%zu", teb_poses_.size());
  if (!optimizer_) {
    optimizer_ = std::make_unique<SparseOptimizer>();
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > BlockSolverX;
    typedef g2o::LinearSolverCSparse<BlockSolverX::PoseMatrixType> LinearSolverType;
    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer_->setAlgorithm(solver);
  }

  // add vertices
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    VertexPose* v = new VertexPose();
    v->setId(static_cast<int>(i));
    v->setEstimate(teb_poses_[i]);
    if (i == 0) v->setFixed(true);
    optimizer_->addVertex(v);
  }

  // unary pose-to-point edges (stay close to sampled path)
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    auto *vtx = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
    if (!vtx) {
      RCLCPP_ERROR(logger_, "optimizeTEB: missing vertex %zu, cannot add pose-to-point edge", i);
      continue;
    }
    EdgePoseToPoint* e = new EdgePoseToPoint();
    e->setVertex(0, vtx);
    e->setMeasurement(teb_poses_[i]);
    e->setInformation(Eigen::Matrix3d::Identity() * 1.0);
    optimizer_->addEdge(e);
  }

  // smoothness edges between consecutive vertices
  for (size_t i = 0; i + 1 < teb_poses_.size(); ++i) {
    auto *v1 = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
    auto *v2 = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i+1)));
    if (!v1 || !v2) {
      RCLCPP_ERROR(logger_, "optimizeTEB: missing vertices for smoothness edge %zu-%zu", i, i+1);
      continue;
    }
    EdgeSmoothness* es = new EdgeSmoothness();
    es->setVertex(0, v1);
    es->setVertex(1, v2);
    Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
    info(0,0) = 8.0; info(1,1) = 8.0; info(2,2) = 1.0; // further reduced smoothness weights
    es->setInformation(info);
    optimizer_->addEdge(es);
  }
  
  // kinematic constraints for differential drive
  for (size_t i = 0; i + 1 < teb_poses_.size(); ++i) {
    auto *v1 = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
    auto *v2 = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i+1)));
    if (!v1 || !v2) continue;
    
    EdgeKinematics* ek = new EdgeKinematics();
    ek->setVertex(0, v1);
    ek->setVertex(1, v2);
    ek->max_vel_x_ = max_velocity_;
    ek->max_vel_theta_ = max_angular_velocity_;
    ek->dt_ = 0.3; // time step between poses
    
    // Moderate penalty for violating kinematic constraints - not too strong
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * 20.0; // reduced from 100.0
    ek->setInformation(info);
    optimizer_->addEdge(ek);
  }
  RCLCPP_INFO(logger_, "optimizeTEB: added %d kinematic constraint edges", teb_poses_.size() - 1);
  
  // obstacle edges - minimal constraints to prevent over-constraining
  int obstacle_edges_added = 0;
  for (size_t i = 1; i < teb_poses_.size() - 1; ++i) { // skip endpoints
    Eigen::Vector2d p(teb_poses_[i].x(), teb_poses_[i].y());
    Eigen::Vector2d obs;
    if (getClosestObstacle(p, obs, 1.0)) {
      double dist = (p - obs).norm();
      if (dist <= obstacle_inflation_ * 0.8) { // only very close obstacles
        auto *v = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
        if (!v) continue;
        
        EdgeObstacle* eo = new EdgeObstacle();
        eo->safety_radius_ = obstacle_inflation_ * 0.6; // smaller safety radius
        eo->setVertex(0, v);
        eo->setMeasurement(obs);
        
        // Moderate penalty - not too strong to avoid over-constraining
        Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 60.0;
        eo->setInformation(info);
        optimizer_->addEdge(eo);
        obstacle_edges_added++;
        
        if (obstacle_edges_added >= 3) break; // very limited obstacle constraints
      }
    }
  }
  RCLCPP_INFO(logger_, "optimizeTEB: added %d minimal obstacle edges", obstacle_edges_added);
  
  optimizer_->initializeOptimization();
  optimizer_->setVerbose(false);
  RCLCPP_INFO(logger_, "optimizeTEB: starting optimization vertices=%zu edges=%zu", optimizer_->vertices().size(), optimizer_->edges().size());
  optimizer_->optimize(2); // minimal iterations to prevent over-optimization

  // write back optimized poses
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    auto v = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
    if (v) teb_poses_[i] = v->estimate();
  }

  optimized_ = true;
  RCLCPP_INFO(logger_, "optimizeTEB: finished optimization (optimized=%d) first=(%f,%f)", optimized_, teb_poses_.front().x(), teb_poses_.front().y());
}

geometry_msgs::msg::Twist ControlCore::computeVelocityCommand() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  if (!hasPath() || !hasOdometry()) {
    return cmd;
  }

  if (teb_poses_.empty()) initializeFromPath();

  RCLCPP_DEBUG(logger_, "computeVelocityCommand: teb_poses=%zu optimized=%d", teb_poses_.size(), optimized_);

  // run optimizer to get a locally optimal trajectory
  optimizeTEB();

  // if goal reached, stop
  if (isGoalReached()) {
    return cmd;
  }

  if (teb_poses_.size() < 2) {
    RCLCPP_WARN(logger_, "computeVelocityCommand: insufficient TEB poses for velocity computation");
    return cmd;
  }

  // Get current robot state
  Eigen::Vector3d robot;
  robot.x() = current_odom_->pose.pose.position.x;
  robot.y() = current_odom_->pose.pose.position.y;
  robot.z() = extractYaw(current_odom_->pose.pose.orientation);

  // Find closest point on TEB trajectory
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    double dx = teb_poses_[i].x() - robot.x();
    double dy = teb_poses_[i].y() - robot.y();
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Use next segment from closest point for velocity computation
  size_t next_idx = std::min(closest_idx + 1, teb_poses_.size() - 1);
  
  // Compute time-optimal velocities based on TEB segment
  Eigen::Vector3d current_pose = teb_poses_[closest_idx];
  Eigen::Vector3d next_pose = teb_poses_[next_idx];
  
  // Check if TEB is making progress - if poses are too close, we might be stuck
  double teb_segment_length = std::hypot(next_pose.x() - current_pose.x(), next_pose.y() - current_pose.y());
  
  if (teb_segment_length < 0.05) { // increased threshold
    consecutive_small_segments_++;
    // TEB might be oscillating - reinitialize trajectory
    RCLCPP_WARN(logger_, "TEB segment too short (%f), consecutive count: %d", teb_segment_length, consecutive_small_segments_);
    
    if (consecutive_small_segments_ >= 3) { // after 3 consecutive small segments, reinitialize
      RCLCPP_WARN(logger_, "TEB appears stuck, reinitializing trajectory");
      teb_poses_.clear();
      consecutive_small_segments_ = 0;
      return cmd; // return zero command and reinitialize next cycle
    }
  } else {
    consecutive_small_segments_ = 0; // reset counter on good progress
  }
  
  // Spatial differences
  double dx = next_pose.x() - current_pose.x();
  double dy = next_pose.y() - current_pose.y();
  double dtheta = next_pose.z() - current_pose.z();
  
  // Normalize angular difference
  while (dtheta > M_PI) dtheta -= 2.0*M_PI;
  while (dtheta < -M_PI) dtheta += 2.0*M_PI;
  
  double segment_length = std::hypot(dx, dy);
  
  // Estimate time for this segment (more conservative to prevent flipping)
  double dt = 0.3; // 300ms lookahead - much slower and more stable
  
  if (segment_length > 1e-6) {
    // Compute desired velocities from TEB trajectory
    double desired_vx = dx / dt;
    double desired_vy = dy / dt;
    double desired_w = dtheta / dt;
    
    // Transform to robot frame
    double cos_theta = std::cos(robot.z());
    double sin_theta = std::sin(robot.z());
    
    double v_forward = desired_vx * cos_theta + desired_vy * sin_theta;
    
    // Apply velocity limits - NO REVERSE MOTION
    v_forward = std::max(0.0, std::min(max_velocity_, v_forward)); // force forward only
    desired_w = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, desired_w));
    
    // Add feedback control to stay on trajectory
    double pos_error_x = current_pose.x() - robot.x();
    double pos_error_y = current_pose.y() - robot.y();
    double heading_error = current_pose.z() - robot.z();
    
    // Normalize heading error
    while (heading_error > M_PI) heading_error -= 2.0*M_PI;
    while (heading_error < -M_PI) heading_error += 2.0*M_PI;
    
    // Feedback gains - much more conservative
    double kp_pos = 0.8;  // reduced from 2.0
    double kp_heading = 1.2;  // reduced from 3.0
    
    // Transform position error to robot frame
    double error_forward = pos_error_x * cos_theta + pos_error_y * sin_theta;
    
    // Combine feedforward and feedback - FORWARD ONLY
    cmd.linear.x = std::max(0.0, v_forward + kp_pos * error_forward); // no reverse
    cmd.angular.z = desired_w + kp_heading * heading_error;
    
    // Final velocity limits - FORWARD ONLY
    cmd.linear.x = std::max(0.0, std::min(max_velocity_, cmd.linear.x)); // no reverse allowed
    cmd.angular.z = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, cmd.angular.z));
  } else {
    // Stationary segment - just correct heading
    double heading_error = current_pose.z() - robot.z();
    while (heading_error > M_PI) heading_error -= 2.0*M_PI;
    while (heading_error < -M_PI) heading_error += 2.0*M_PI;
    
    cmd.linear.x = 0.0;
    cmd.angular.z = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, 1.5 * heading_error));  // reduced from 3.0
  }

  return cmd;
}

bool ControlCore::isGoalReached() const {
  if (!current_path_ || current_path_->poses.empty() || !current_odom_) {
    return false;
  }

  // Get robot position
  double robot_x = current_odom_->pose.pose.position.x;
  double robot_y = current_odom_->pose.pose.position.y;
  
  // Get goal position (last pose in path)
  const auto& goal = current_path_->poses.back();
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  
  // Calculate distance to goal
  double dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);
  
  // Goal is reached if within 0.2m - adjusted for new conservative parameters
  return dist_to_goal < 0.2;
}