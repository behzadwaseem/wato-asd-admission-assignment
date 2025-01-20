#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

  private:
    rclcpp::Logger logger_;

};

}  

#endif  
/*

#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Functions for generating & updating the costmap:
    void initCostmap(int width, int height, double resolution, double inflation_radius, geometry_msgs::msg::Pose origin);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;


  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr cm_data_;
    int width_;
    int height_;
    double resolution_;
    double inflation_radius_;
    geometry_msgs::msg::Pose origin;

};
}  

#endif  

*/