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
  double yaw = extractYaw(current_odom_->pose.pose.orientation);
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

  // sample along path
  Eigen::Vector3d last = teb_poses_.empty() ? Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0) : teb_poses_.back();
  double acc_dist = 0.0;
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double x = current_path_->poses[i].pose.position.x;
    double y = current_path_->poses[i].pose.position.y;
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
    // ensure at least two points
    teb_poses_.push_back(Eigen::Vector3d(current_path_->poses.back().pose.position.x,
                                         current_path_->poses.back().pose.position.y, 0.0));
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
    RCLCPP_WARN(logger_, "optimizeTEB: optimizer was null, creating new one");
    optimizer_ = std::make_unique<SparseOptimizer>();
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > BlockSolverX;
    typedef g2o::LinearSolverCSparse<BlockSolverX::PoseMatrixType> LinearSolverType;
    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer_->setAlgorithm(solver);
  } else {
    RCLCPP_DEBUG(logger_, "optimizeTEB: reusing existing optimizer instance");
  }

  // add vertices
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    VertexPose* v = new VertexPose();
    v->setId(static_cast<int>(i));
    v->setEstimate(teb_poses_[i]);
    if (i == 0) v->setFixed(true);
    optimizer_->addVertex(v);
    auto *got = optimizer_->vertex(static_cast<int>(i));
  }
  RCLCPP_INFO(logger_, "optimizeTEB: after vertex addition vertices=%zu", optimizer_->vertices().size());

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
  RCLCPP_INFO(logger_, "optimizeTEB: after pose edges edges=%zu", optimizer_->edges().size());

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
    info(0,0) = 10.0; info(1,1) = 10.0; info(2,2) = 1.0;
    es->setInformation(info);
    optimizer_->addEdge(es);
  }
  // obstacle edges
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    Eigen::Vector2d p(teb_poses_[i].x(), teb_poses_[i].y());
    Eigen::Vector2d obs;
    if (getClosestObstacle(p, obs, 2.0)) {
      double dist = (p - obs).norm();
      if (dist <= (obstacle_inflation_ + 0.5)) {
        auto *v = dynamic_cast<VertexPose*>(optimizer_->vertex(static_cast<int>(i)));
        if (!v) {
          RCLCPP_ERROR(logger_, "optimizeTEB: missing vertex %zu for obstacle edge", i);
          continue;
        }
        EdgeObstacle* eo = new EdgeObstacle();
        eo->safety_radius_ = obstacle_inflation_;
        eo->setVertex(0, v);
        eo->setMeasurement(obs);
        Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 80.0; // strong penalty
        eo->setInformation(info);
        optimizer_->addEdge(eo);
      }
    }
  }
  RCLCPP_INFO(logger_, "optimizeTEB: after obstacle edges edges=%zu", optimizer_->edges().size());

  optimizer_->initializeOptimization();
  optimizer_->setVerbose(false);
  RCLCPP_INFO(logger_, "optimizeTEB: starting optimization vertices=%zu edges=%zu", optimizer_->vertices().size(), optimizer_->edges().size());
  optimizer_->optimize(8);

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

  // use first planned segment to compute command
  Eigen::Vector3d robot;
  robot.x() = current_odom_->pose.pose.position.x;
  robot.y() = current_odom_->pose.pose.position.y;
  robot.z() = extractYaw(current_odom_->pose.pose.orientation);

  // select a target ahead of robot: skip poses that are too close to current robot pose
  size_t target_idx = 0;
  double min_target_dist = 0.08; // meters
  for (size_t k = 1; k < teb_poses_.size(); ++k) {
    double dxk = teb_poses_[k].x() - robot.x();
    double dyk = teb_poses_[k].y() - robot.y();
    double dk = std::hypot(dxk, dyk);
    RCLCPP_DEBUG(logger_, "computeVelocityCommand: checking teb[%zu] dist=%f", k, dk);
    if (dk > min_target_dist) { target_idx = k; break; }
  }
  // fallback to last if all are too close; prefer the one with max distance
  if (target_idx == 0) {
    size_t furthest = 0;
    double maxd = 0.0;
    for (size_t k = 0; k < teb_poses_.size(); ++k) {
      double dk = std::hypot(teb_poses_[k].x() - robot.x(), teb_poses_[k].y() - robot.y());
      if (dk > maxd) { maxd = dk; furthest = k; }
    }
    if (maxd > 0.0) target_idx = furthest;
  }

  Eigen::Vector3d target = teb_poses_[target_idx];
  double dx = target.x() - robot.x();
  double dy = target.y() - robot.y();
  double dist = std::hypot(dx, dy);
  double angle_to_target = std::atan2(dy, dx);
  double angular_error = angle_to_target - robot.z();
  // normalize
  while (angular_error > M_PI) angular_error -= 2.0*M_PI;
  while (angular_error < -M_PI) angular_error += 2.0*M_PI;

  RCLCPP_INFO(logger_, "computeVelocityCommand: robot=(%f,%f,%f) target_idx=%zu target=(%f,%f) dist=%f ang_err=%f", robot.x(), robot.y(), robot.z(), target_idx, target.x(), target.y(), dist, angular_error);

  // if target is essentially coincident with robot but goal not reached, nudge forward
  if (dist < 1e-3 && !isGoalReached()) {
    RCLCPP_WARN(logger_, "computeVelocityCommand: target coincident with robot (dist=%f); applying small nudge", dist);
    double v_nudge = std::min(0.06, max_velocity_);
    cmd.linear.x = v_nudge;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(logger_, "computeVelocityCommand: nudge cmd linear_x=%f", cmd.linear.x);
    return cmd;
  }

  // simple P controllers
  double kv = 0.8; // velocity gain
  double kw = 1.5; // angular gain

  double v = std::min(max_velocity_, kv * dist);
  double w = std::min(max_angular_velocity_, std::max(-max_angular_velocity_, kw * angular_error));

  // if heading large, rotate first
  if (std::abs(angular_error) > 0.6) v = 0.0;

  cmd.linear.x = v;
  cmd.angular.z = w;

  RCLCPP_INFO(logger_, "computeVelocityCommand: computed cmd linear_x=%f angular_z=%f", cmd.linear.x, cmd.angular.z);

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
  
  // Goal is reached if within 0.2m
  return dist_to_goal < 0.2;
}