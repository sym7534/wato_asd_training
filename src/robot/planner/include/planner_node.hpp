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

  bool operator==(const CellIndex &otherCell)
  {
    return(x == otherCell.x && y == otherCell.y);
  }

  bool operator!=(const CellIndex &otherCell)
  {
    return (!(*this == otherCell));
  }
};

struct CompareFScore
{
  bool operator()(const AStarNode &x, const AStarNode &y)
  {
    return(x.fScore > y.fScore);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &index)
  {
    return(std::hash<int>()(index.x)^(std::hash<int>()(index.y) <<1));
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



class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    NodeState state_ = NodeState::WAITING_FOR_GOAL;
    robot::PlannerCore planner_;

    // latest goal object
    geometry_msgs::msg::PointStamped latestGoal;

    // path message to be published

    nav_msgs::msg::Path pathMessage;

    bool hasNewGoal;
};

#endif 
