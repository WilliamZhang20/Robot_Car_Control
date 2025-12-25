#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), width_(0), height_(0), resolution_(0.1) {}

void CostmapCore::initializeCostmap(int width, int height, double resolution) {
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  costmap_.assign(width_ * height_, 0); // free space
}

bool CostmapCore::convertToGrid(double x, double y, int &x_grid, int &y_grid) {
  // Assume map centered at (0,0)
  x_grid = static_cast<int>((x / resolution_) + width_ / 2);
  y_grid = static_cast<int>((y / resolution_) + height_ / 2);

  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_)
    return false;
  return true;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    costmap_[y_grid * width_ + x_grid] = 100; // occupied
  }
}

}