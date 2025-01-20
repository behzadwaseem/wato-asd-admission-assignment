
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // for lidar topic
#include "nav_msgs/msg/occupancy_grid.hpp" // for occupancy grid
#include "costmap_node.hpp"

// ROS COMMUNICATION FOR COSTMAP WILL GO IN THIS FILE.

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
// CostmapNode::CostmapNode() : Node("costmap_node"), costmap_(get_logger())
{
  // Declare node paramters:
  declareCostmapParamaters();

  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  // Initialize subscriber for /lidar topic:
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidar_topic, 10, std::bind(&CostmapNode::LidarCallback, this, std::placeholders::_1));

  //costmap_.initCostmap(cm_width, cm_height, cm_resolution, cm_inflation_radius, cm_origin);

}

void CostmapNode::declareCostmapParamaters()
{
  /*
    Declares paramaters for the costmap node (eg. lidar, costmap, costmap features).
  */

  // Lidar:
  this->declare_parameter<std::string>("lidar_topic", "/lidar");
  lidar_topic = this->get_parameter("lidar_topic").as_string();

  // Costmap:
  this->declare_parameter<std::string>("cm_topic", "/costmap");
  cm_topic = this->get_parameter("cm_topic").as_string();

  // Costmap Features:
  this->declare_parameter<int>("cm_width", 100);
  cm_width = this->get_parameter("cm_width").as_int();
  
  this->declare_parameter<int>("cm_height", 100);
  cm_height = this->get_parameter("cm_height").as_int();
  
  this->declare_parameter<double>("cm_resolution", 0.1);
  cm_resolution = this->get_parameter("cm_resolution").as_double();

  this->declare_parameter<double>("cm_inflation_radius", 1.0);
  cm_inflation_radius = this->get_parameter("cm_inflation_radius").as_double();

  // Initialize robot at bottom-left corner:
  this->declare_parameter<double>("cm_origin_x", -5.0);
  cm_origin.position.x = this->get_parameter("cm_origin_x").as_double();

  this->declare_parameter<double>("cm_origin_y", -5.0);
  cm_origin.position.y = this->get_parameter("cm_origin_y").as_double();

  this->declare_parameter<double>("cm_origin_z", 0.0);
  cm_origin.position.z = this->get_parameter("cm_origin_z").as_double();

  this->declare_parameter<double>("cm_origin_w", 1.0);
  cm_origin.orientation.w = this->get_parameter("cm_origin_w").as_double();
}

void CostmapNode::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  /*
    Processes and recieves messages published to /lidar topic. (for now, i'm just going to use this to check that i've subscribed properly)
  */
  
  // Check if the 'ranges' array is not empty
  if (!lidar_msg->ranges.empty())
  {
    // Log the size of the ranges array
    RCLCPP_INFO(this->get_logger(), "Received LaserScan with %zu ranges", lidar_msg->ranges.size());

    // Optionally, log the first few values in the ranges array
    RCLCPP_INFO(this->get_logger(), "First range: %f", lidar_msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "Last range: %f", lidar_msg->ranges[lidar_msg->ranges.size() - 1]);

    // Optionally, check for invalid values (NaN or Inf)
    for (size_t i = 0; i < lidar_msg->ranges.size(); ++i)
    {
      if (std::isnan(lidar_msg->ranges[i]) || std::isinf(lidar_msg->ranges[i]))
      {
        RCLCPP_WARN(this->get_logger(), "Invalid range value at index %zu: %f", i, lidar_msg->ranges[i]);
      }
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received empty LaserScan data!");
  }

  // Check other fields of the LaserScan message for consistency
  RCLCPP_INFO(this->get_logger(), "Scan start angle: %f, Scan end angle: %f, Increment: %f",
              lidar_msg->angle_min, lidar_msg->angle_max, lidar_msg->angle_increment);
}

// Define the timer to publish a message every 500ms (FROM TUTORIAL -- REMOVE)
void CostmapNode::publishMessage()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}