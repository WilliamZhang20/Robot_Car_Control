#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Initialize costmap dimensions: 200x200 cells, resolution 0.1 m → 20m × 20m area
  costmap_.initializeCostmap(200, 200, 0.1);
}

void CostmapNode::inflateObstacles() {
  auto &grid = costmap_.data(); // explicitly splitting assignment and deep copy in 2 steps
  int width = costmap_.getWidth();
  int height = costmap_.getHeight();
  
  std::vector<int8_t> inflated = grid; // copy from ref

  double resolution = costmap_.getResolution();
  double inflation_radius = 1.0; // 1 meter
  int cells_radius = static_cast<int>(inflation_radius / resolution);
  int max_cost = 100;
  
  // traverse grid in 2D manner
  for(int y=0; y<height; y++) {
    for(int x=0; x<width; x++) {
      if (grid[y * width + x] == 100) { // obstacle
        for (int dy = -cells_radius; dy <= cells_radius; dy++) {
          for (int dx = -cells_radius; dx <= cells_radius; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
              continue;

            double dist = std::sqrt(dx * dx + dy * dy) * resolution;
            if (dist <= inflation_radius) {
              int cost = static_cast<int>(max_cost * (1.0 - dist / inflation_radius));
              inflated[ny * width + nx] = std::max<int8_t>(inflated[ny * width + nx], cost);
            }
          }
        }
      }
    }
  }

  grid.swap(inflated);
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  msg.info.resolution = costmap_.getResolution();
  msg.info.width = costmap_.getWidth();
  msg.info.height = costmap_.getHeight();
  msg.info.origin.position.x = - (msg.info.width * msg.info.resolution) / 2.0;
  msg.info.origin.position.y = - (msg.info.height * msg.info.resolution) / 2.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = costmap_.data();
  costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    costmap_.initializeCostmap(costmap_.getWidth(), costmap_.getHeight(), costmap_.getResolution());
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            int gx, gy;
            if (costmap_.convertToGrid(x, y, gx, gy)) {
              costmap_.markObstacle(gx, gy);
            }
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}