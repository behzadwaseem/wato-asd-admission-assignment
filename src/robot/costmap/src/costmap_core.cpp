#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

}



/*
#include "costmap_core.hpp"


// CORE FUNCTIONS OF COSTMAP WILL GO IN THIS FILE.

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
        cm_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    }

    void CostmapCore::initCostmap(int width, int height, double resolution, double inflation_radius, geometry_msgs::msg::Pose origin) {
        /*
            Initializes the costmap based on specified paramaters. Sets all starting cell values to 0.
        */
        
//         cm_data_->header.frame_id = "map";
//         cm_data_->info.width = width;
//         cm_data_->info.height = height;
//         cm_data_->info.resolution = resolution;
//         cm_data_->info.origin = origin;
        
//         cm_data_->data.resize(width*height, 0);

//         // Inflation
//         inflation_radius_ = inflation_radius;
//         // TODO - MAKE CALL TO INFLATE_CM_OBSTACLES FUNCTION . . .

//         RCLCPP_INFO(logger_, "Costmap initialized with size %dx%d and resolution %.2f", width, height, resolution);
//     }


// }
