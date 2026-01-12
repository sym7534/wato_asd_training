#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore mapMemory;

    // subscriptions + publisher topics
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;

    // timer
    rclcpp::TimerBase::SharedPtr timer;

    // latest costmap object
    nav_msgs::msg::OccupancyGrid::SharedPtr latestCostmap;

    // current robot position/ prev robot position
    double curX;
    double curY;
    double curYaw;
    bool havePose;
    double lastUpdateX;
    double lastUpdateY;
    bool haveLastUpdate;

    // map parameters
    double updateDistance;

    int updateIntervalMs;

    double globalResolution;
    int globalWidth;
    int globalHeight;
    double globalOriginX;
    double globalOriginY;

    std::string globalFrameId;

    // callbacks
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // robot yaw getter
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat);
};

#endif 
