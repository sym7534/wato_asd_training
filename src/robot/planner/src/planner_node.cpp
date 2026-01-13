#include "planner_node.hpp"

#include <cmath>


PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) 
{
  // subscribers
  mapSub = this->create_subscription<nav:msgs::msg::OccupancyGrid>("/map",10,std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goalSub = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odomSub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // publisher
  pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  planningTimer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// helper function to check if we're at the goal
void PlannerNode::atGoal()
{
  
}

void PlannerNode::waitForGoal(const geometry_msgs::msg::PointStamped goal)
{
  if(!latestGoal)
  {
    return;
  }


}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};