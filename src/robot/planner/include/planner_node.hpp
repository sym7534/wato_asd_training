#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "planner_core.hpp"

#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <functional>

enum class NodeState
{
  WAITING_FOR_GOAL,
  WAITING_FOR_ROBOT_TO_REACH_GOAL
};

struct CellIndex
{
  double x;
  double y;

  CellIndex(double a, double b)
  {
    x = a;
    y = b;
  } 
  
  CellIndex()
  {
    x = 0;
    y = 0;
  }

  bool operator==(const CellIndex &otherCell) const
  {
    return(x == otherCell.x && y == otherCell.y);
  }

  bool operator!=(const CellIndex &otherCell) const
  {
    return (!(*this == otherCell));
  }
};

struct AStarNode;

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &index) const
  {
    return(std::hash<int>()(static_cast<int>(index.x))^(std::hash<int>()(static_cast<int>(index.y)) <<1));
  }
};

struct AStarNode
{
  CellIndex index;
  CellIndex parentIndex;
  double gScore; // distance from starting node
  double hScore; // distance from end node

  double fScore; // gscore + hscore

  AStarNode(CellIndex indx, CellIndex parent, double g, double h, double f)
  {
    index = indx;
    parentIndex = parent;
    gScore = g;
    hScore = h;
    fScore = f;
  }

  AStarNode()
  {
    index.x = 0;
    index.y = 0;
    gScore = INFINITY;
    hScore = INFINITY;
    fScore = INFINITY;
  }
};

struct CompareFScore
{
  bool operator()(const AStarNode &x, const AStarNode &y) const
  {
    return(x.fScore > y.fScore);
  }
};


class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:

    const int mapSize = 300;
    const int mapRes = 0.1;

    // state
    NodeState state_ = NodeState::WAITING_FOR_GOAL;

    robot::PlannerCore planner_;

    // subscribers/publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publishedPath;

    // timer
    rclcpp::TimerBase::SharedPtr planningTimer;

    // objects
    nav_msgs::msg::OccupancyGrid map;
    geometry_msgs::msg::PointStamped latestGoal;
    nav_msgs::msg::Odometry currentPose;

    // path message to be published

    nav_msgs::msg::Path pathMessage;

    bool hasNewGoal;

    // callbacks + helpers
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool atGoal();
    void pathPlanner();
    bool isBlockedCell(const CellIndex &idx, int gCols, int gRows);
    double distance(const CellIndex &a, const CellIndex &b);
};

#endif 
