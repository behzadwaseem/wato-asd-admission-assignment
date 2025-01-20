#include "costmap_core.hpp"
#include <cmath>

// CORE FUNCTIONS OF COSTMAP WILL GO IN THIS FILE.

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger)
    {
        cm_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    }

    void CostmapCore::initCostmap(int width, int height, double resolution, double inflation_radius, geometry_msgs::msg::Pose origin)
    {
        /*
            Initializes the costmap based on specified paramaters. Sets all starting cell values to 0.
        */

        // Set CostmapCore variables:
        cm_data_->header.frame_id = "map";
        cm_data_->info.width = width;
        cm_data_->info.height = height;
        cm_data_->info.resolution = resolution;
        cm_data_->info.origin = origin;

        cm_data_->data.resize(width * height, 0); // initialize grid with 0's

        inflation_radius_ = inflation_radius;
        inflation_grid_cells_ = std::ceil(inflation_radius / resolution); // convert inflation radius from meters to grid cells

        RCLCPP_INFO(logger_, "Costmap initialized with size %dx%d and resolution %.2f", width, height, resolution);
    }

    void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) const
    {
        /*
            Updates the costmap based on incoming lidar data from robot.
        */

        // Extract data from lidar scan:
        cm_data_->header.stamp = lidar_msg->header.stamp; // update timestamp of costmap to match lidar scan
        double angle_min = lidar_msg->angle_min;
        double angle_increment = lidar_msg->angle_increment;
        double range_size = lidar_msg->ranges.size();
        double max_range = lidar_msg->range_max;

        // Build costmap from lidar data:
        for (size_t i; i < range_size; i++)
        {
            double curr_range = lidar_msg->ranges[i];

            // Only operate on valid ranges:
            if (!(curr_range < lidar_msg->range_min) || !(curr_range > lidar_msg->range_max) || std::isfinite(curr_range))
            {

                // Calculate Cartesian coords:
                double angle = (i * angle_increment) + angle_min;
                double x = curr_range * std::cos(angle);
                double y = curr_range * std::sin(angle);

                // Convert Cartesian coords to grid cell indices:
                int grid_x = static_cast<int>((x - cm_data_->info.origin.position.x) / cm_data_->info.resolution);
                int grid_y = static_cast<int>((y - cm_data_->info.origin.position.y) / cm_data_->info.resolution);

                // Validate grid cell indices:
                if (grid_x >= 0 && grid_x < static_cast<int>(cm_data_->info.width) && grid_y >= 0 && grid_y < static_cast<int>(cm_data_->info.height))
                {

                    // Mark the cell as an obstacle:
                    int cell_index = grid_y * cm_data_->info.width + grid_x;
                    cm_data_->data[cell_index] = 100; // indicate cell represents an obstacle (high cost)

                    // Inflate area around obstacle cell to account for sensor imprecision:
                    inflateCostmapObstacle(grid_x, grid_y);
                }
            }
        }

        // Reset costmap:
        std::fill(cm_data_->data.begin(), cm_data_->data.end(), 0);
    }

    void CostmapCore::inflateCostmapObstacle(int obstacle_x, int obstacle_y) const
    {
        /*
            Inflates areas around obstacles detected by lidar to account for sensor imprecision.
        */

        // Loop through the cells within the inflation radius:
        for (int dx = -inflation_grid_cells_; dx <= inflation_grid_cells_; ++dx)
        {
            for (int dy = -inflation_grid_cells_; dy <= inflation_grid_cells_; ++dy)
            {
                // Calculate cell coordinates:
                int x = obstacle_x + dx;
                int y = obstacle_y + dy;

                // Skip cells that are out of bounds:
                if (isWithinBounds(x, y))
                {
                    // Compute the distance from the obstacle's origin
                    double distance = calculateDistance(dx, dy);

                    // Inflate only if within the inflation radius
                    if (distance <= inflation_radius_)
                    {
                        applyInflationCost(x, y, distance);
                    }
                }
            }
        }
    }

    bool CostmapCore::isWithinBounds(int x, int y) const
    {
        /*
            Helper function to check if a cell is within bounds
        */
        return x >= 0 && x < static_cast<int>(cm_data_->info.width) && y >= 0 && y < static_cast<int>(cm_data_->info.height);
    }

    double CostmapCore::calculateDistance(int dx, int dy) const
    {
        /*
            Helper function to calculate the Euclidean distance
        */
        return std::hypot(dx * cm_data_->info.resolution, dy * cm_data_->info.resolution);
    }

    void CostmapCore::applyInflationCost(int x, int y, double distance) const
    {
        /*
            Helper function to apply the inflation cost
        */
        int index = y * cm_data_->info.width + x;

        // Calculate the inflation cost (linear decay)
        int cost = static_cast<int>((1.0 - distance / inflation_radius_) * 99);

        // Update the costmap with the maximum of the current cost and new cost
        cm_data_->data[index] = std::max(static_cast<int8_t>(cost), cm_data_->data[index]);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
    {
        return cm_data_;
    }
}

// error - "Error response from daemon: network 39c624a81fc992a5ea0fcd5a3154a6028ca971612423be7e4f89a9551c10d52d not found"