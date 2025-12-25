#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger), 
    min_lookahead_distance_(0.3),
    max_lookahead_distance_(2.0),
    lookahead_time_(1.5),
    max_linear_speed_(1.0),
    min_linear_speed_(0.1),
    max_angular_speed_(1.0),
    goal_tolerance_(0.2) {}

void ControlCore::processPath(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
}

void ControlCore::processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odometry_ = msg;
}

geometry_msgs::msg::Twist ControlCore::generateCommand() {
    geometry_msgs::msg::Twist cmd_vel;

    if (!current_path_) {
        RCLCPP_WARN(logger_, "No path received!");
        return cmd_vel;
    }

    if (!current_odometry_) {
        RCLCPP_WARN(logger_, "No odometry received!");
        return cmd_vel;
    }

    if (current_path_->poses.empty()) {
        RCLCPP_WARN(logger_, "Path is empty!");
        return cmd_vel;
    }

    double robot_x = current_odometry_->pose.pose.position.x;
    double robot_y = current_odometry_->pose.pose.position.y;
    double robot_yaw = extractYaw(current_odometry_->pose.pose.orientation);
    
    // Get current velocity for regulation
    double current_speed = std::hypot(current_odometry_->twist.twist.linear.x, 
                                     current_odometry_->twist.twist.linear.y);

    // Check if goal is reached
    const auto& goal = current_path_->poses.back();
    double dist_to_goal = computeDistance(robot_x, robot_y, 
                                         goal.pose.position.x, goal.pose.position.y);
    
    if (dist_to_goal < goal_tolerance_) {
        RCLCPP_INFO(logger_, "Goal reached! Distance: %.2f", dist_to_goal);
        return cmd_vel; // Stop at goal
    }

    // Calculate adaptive lookahead distance based on speed
    double adaptive_lookahead = std::max(min_lookahead_distance_, 
                                        std::min(max_lookahead_distance_, 
                                               current_speed * lookahead_time_));

    auto lookahead_point = findLookaheadPoint(robot_x, robot_y, adaptive_lookahead);
    if (!lookahead_point) {
        RCLCPP_WARN(logger_, "No lookahead point found!");
        return cmd_vel;
    }

    // Calculate curvature to lookahead point
    double dx = lookahead_point->pose.position.x - robot_x;
    double dy = lookahead_point->pose.position.y - robot_y;
    double lookahead_dist = std::hypot(dx, dy);
    
    // Transform to robot frame
    double alpha = std::atan2(dy, dx) - robot_yaw;
    
    // Calculate curvature (1/radius)
    double curvature = 2.0 * std::sin(alpha) / lookahead_dist;
    
    // Regulate speed based on curvature (slow down for sharp turns)
    double target_speed = max_linear_speed_ / (1.0 + std::abs(curvature) * 2.0);
    target_speed = std::max(min_linear_speed_, target_speed);
    
    // Further reduce speed when approaching goal
    if (dist_to_goal < 1.0) {
        target_speed *= (dist_to_goal / 1.0);
        target_speed = std::max(min_linear_speed_ * 0.5, target_speed);
    }
    
    // Calculate angular velocity
    double angular_vel = curvature * target_speed;
    angular_vel = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_vel));

    cmd_vel.linear.x = target_speed;
    cmd_vel.angular.z = angular_vel;

    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(double robot_x, double robot_y, double lookahead_distance) {
    if (current_path_->poses.empty()) {
        return std::nullopt;
    }
    
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dist = computeDistance(robot_x, robot_y, current_path_->poses[i].pose.position.x, current_path_->poses[i].pose.position.y);
        if (dist >= lookahead_distance) {
            return current_path_->poses[i];
        }
    }
    
    // If no point is far enough, use the last point in the path
    return current_path_->poses.back();
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  