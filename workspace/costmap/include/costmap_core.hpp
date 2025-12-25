#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(int width, int height, double resolution);
    bool convertToGrid(double x, double y, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    std::vector<int8_t>& data() { return costmap_; }
      
  private:
    rclcpp::Logger logger_;
    int width_;
    int height_;
    double resolution_;
    std::vector<int8_t> costmap_;
};

}  

#endif  