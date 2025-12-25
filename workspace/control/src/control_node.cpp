#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::TebOptimalPlanner(this->get_logger())) {
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&ControlNode::mapCallback, this, std::placeholders::_1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ControlNode::controlLoop, this));  // Increased from 10ms to 50ms (20Hz) for trajectory convergence
}

void ControlNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    control_.setOccupancyGrid(msg);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    control_.setGlobalPath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    control_.setOdometry(msg);
}

void ControlNode::controlLoop() {
    try {
        // Safety check - don't run TEB optimization without valid data
        if (!control_.hasPath() || !control_.hasOdometry()) {
            RCLCPP_DEBUG(this->get_logger(), "Missing data - path: %s, odom: %s", 
                         control_.hasPath() ? "yes" : "no",
                         control_.hasOdometry() ? "yes" : "no");
            // Publish zero velocity when no data available
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        // Check if goal is reached - stop control loop if so
        if (control_.isGoalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            geometry_msgs::msg::Twist cmd_vel;  // Zero velocity
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }
        
        control_.optimizeTEB(
            15,       // iterations_innerloop 
            3,       // iterations_outerloop
            false,   // compute_cost_afterwards
            3.0,     // obst_cost_scale
            1.0,     // viapoint_cost_scale
            false    // alternative_time_cost
        );
        
        geometry_msgs::msg::Twist cmd_vel = control_.computeVelocityCommand();
        RCLCPP_DEBUG(this->get_logger(), "Velocity command: linear.x=%.3f, angular.z=%.3f", 
                     cmd_vel.linear.x, cmd_vel.angular.z);

        cmd_vel_pub_->publish(cmd_vel);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Control loop exception: %s", e.what());
        // Publish zero velocity on error
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown control loop exception");
        // Publish zero velocity on error
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}