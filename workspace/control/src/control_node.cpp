#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
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
    RCLCPP_INFO(this->get_logger(), "mapCallback: received map width=%d height=%d res=%f", msg->info.width, msg->info.height, msg->info.resolution);
    control_.setOccupancyGrid(msg);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "pathCallback: received path with %zu poses", msg->poses.size());
    control_.setGlobalPath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw = 0.0; // extract yaw for log
    {
      auto &o = msg->pose.pose.orientation;
      double siny_cosp = 2.0 * (o.w * o.z + o.x * o.y);
      double cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z);
      yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    RCLCPP_DEBUG(this->get_logger(), "odomCallback: x=%f y=%f yaw=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    control_.setOdometry(msg);
}

void ControlNode::controlLoop() {
    RCLCPP_DEBUG(this->get_logger(), "controlLoop: hasPath=%d hasOdom=%d hasMap=%d", control_.hasPath(), control_.hasOdometry(), control_.hasOccupancyGrid());
    if (!control_.hasOdometry() || !control_.hasPath()) return;

    auto cmd = control_.computeVelocityCommand();

    // If goal reached ensure we publish zero velocity
    if (control_.isGoalReached()) {
      RCLCPP_INFO(this->get_logger(), "controlLoop: goal reached - publishing stop");
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "controlLoop: publishing cmd linear_x=%f angular_z=%f", cmd.linear.x, cmd.angular.z);
    cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}