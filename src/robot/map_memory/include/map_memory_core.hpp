#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    MapMemoryCore(const rclcpp::Logger& logger); // constructor

    // intiailizes global map (but we use parameters this time!)
    void initializeGlobalMap(double resolution, int cols, int rows, double originX, double originY);

    // merges the latest local costmap into the global map
    void updateGlobalMap(
      const nav_msgs::msg::OccupancyGrid::SharedPtr localCostmap,
      double robotX,
      double robotY,
      double robotYaw
    );

    /* for reference:
    Aggregate Costmaps into the Global Map
Maintain a global OccupancyGrid that represents the map of the environment.
When the robot moves 1.5 meters, integrate the most recent costmap into the global map:
Transform the costmap into the global frame using the robot’s current position and orientation.
Update the global map by merging the transformed costmap into it, prioritizing new data over old data.
    */

    // returns the global map so the node can publish it
    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap();

  private:
    rclcpp::Logger logger;
    nav_msgs::msg::OccupancyGrid::SharedPtr globalMap;
};

}  

#endif  
