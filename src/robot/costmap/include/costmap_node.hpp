#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // for lidar topic
#include "nav_msgs/msg/occupancy_grid.hpp" // for occupancy grid
#include "nav_msgs/msg/occupancy_grid.hpp" // for occupancy grid

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();
  void declareCostmapParamaters();

  void publishMessage();

  // Place callback function here
private:
  void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) const;

  robot::CostmapCore costmap_;

  // Lidar:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  std::string lidar_topic_;

  // Costmap
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cm_pub_;
  std::string cm_topic_;

  // Costmap Features:
  int cm_width_;
  int cm_height_;
  geometry_msgs::msg::Pose cm_origin_;
  double cm_resolution_;
  double cm_inflation_radius_;

  // Place these constructs here
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif
