#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

// https://docs.ros.org/en/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html

MapMemoryNode::MapMemoryNode() : Node("map_memory"), mapMemory(robot::MapMemoryCore())
{
  // odometry data
  curX = 0.0;
  curY = 0.0;
  curYaw = 0.0;

  // boolean used to check whether we have odometry data or not
  hasOdom = false;

  // update tracking variables
  lastUpdateX = 0.0;
  lastUpdateY = 0.0;
  haveLastUpdate = false;

  // update distance/freq
  updateDistance = 1.5;
  updateTime = 1000; // TUNE THESE

  // global map parameters
  gRes = 0.1;
  gWidth = 300;
  gHeight = 300;

  gOriginX = -(static_cast<double>(gWidth) * gRes) / 2.0;
  gOriginY = -(static_cast<double>(gHeight) * gRes) / 2.0;

  globalFrameId = "map";

  // create the global map storage
  mapMemory.initializeGlobalMap(gRes, gWidth, gHeight, gOriginX, gOriginY);

  // subscribe to the local costmap from /costmap topic
  costmapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap",10,std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  // subscribe to /odom/filtered topic
  odomSub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // publisher to /map topic
  mapPublish = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // timer to limit how often we fuse maps
  timer = this->create_wall_timer(std::chrono::milliseconds(updateTime),std::bind(&MapMemoryNode::timerCallback, this));
  // yo: gurt
  // std: chrono
} 

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // save the most recent local costmap for the next update
  latestCostmap = msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // update the robot position based on odometry data
  curX = msg->pose.pose.position.x;
  curY = msg->pose.pose.position.y;
  curYaw = getYawFromQuaternion(msg->pose.pose.orientation);
  hasOdom = true;
}

void MapMemoryNode::timerCallback()
{
  bool shouldUpdate = false;

  if (!latestCostmap || !hasOdom)
  {
    return;
  }

  // first time update or distance-based update
  if (!haveLastUpdate)
  {
    shouldUpdate = true;
  }
  else
  {
    double dx = curX - lastUpdateX;
    double dy = curY - lastUpdateY;
    double distanceMoved = std::sqrt((dx * dx) + (dy * dy));
    if (distanceMoved >= updateDistance)
    {
      shouldUpdate = true;
    }
  }

  if (shouldUpdate == true)
  {
    // store the point we are updating from, the update the global map
    lastUpdateX = curX;
    lastUpdateY = curY;
    haveLastUpdate = true;

    mapMemory.updateGlobalMap(latestCostmap, curX, curY, curYaw);
  }

  // publish global map
  nav_msgs::msg::OccupancyGrid::SharedPtr globalMap = mapMemory.getGlobalMap();
  
  globalMap->header.stamp = this->get_clock()->now(); // timestamp of globalmap
  globalMap->header.frame_id = globalFrameId; // same frame id

  mapPublish->publish(*globalMap);
}

// https://stackoverflow.com/questions/74464503/quaternion-to-yaw-to-heading
double MapMemoryNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  // convert quaternion to yaw ()
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
