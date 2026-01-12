#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"
#include <cmath>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishCostmap();

    static const int SIZE = 300;
    double costmap[SIZE][SIZE];

  private:
    robot::CostmapCore costmapCore;

    double resolution = 0.1;
    double origin_x = -(SIZE * resolution) / 2.0;
    double origin_y = -(SIZE * resolution) / 2.0;
    double inflation_radius = 1.4;
    double max_cost = 100.0;

    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publishedCostmap;

    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;
    
    void initializeCostmap();
    void topicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void inflateCostmap();
};
 
#endif 
