#include "planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <vector>

#include <cmath>

#include "geometry_msgs/msg/point_stamped.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  hasNewGoal = false;

  // subscribers
  mapSubscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10,std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goalSubscription = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point",10,std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // publisher
  publishedPath = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // timer for replanning + goal checking
  planningTimer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&PlannerNode::timerCallback, this));
}

// get map,
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMessage)
{
  map = *mapMessage;

  if (state_ == NodeState::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    pathPlanner();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goalMessage)
{
  latestGoal = *goalMessage;
  hasNewGoal = true;
  state_ = NodeState::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  // update path immediately after getting new goal
  pathPlanner();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMessage)
{
  currentPose = *odomMessage;
}

// state "switching"
void PlannerNode::timerCallback()
{
  // skip timer if robot is pathfinding to goal
  if (state_ != NodeState::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    return;
  }

  // if robot reaches goal, wait for new goal
  if (atGoal())
  {
    state_ = NodeState::WAITING_FOR_GOAL;
    return;
  }

  // aagghhgi is this even needed
  pathPlanner();
}

// check for if we're at the goal
bool PlannerNode::atGoal()
{
  if (!hasNewGoal)
  {
    return false;
  }

  double dx = latestGoal.point.x - currentPose.pose.pose.position.x;
  double dy = latestGoal.point.y - currentPose.pose.pose.position.y;

  double distanceToGoal = std::sqrt((dx * dx) + (dy * dy));

  return(distanceToGoal < 0.5);
  // 0.5 -> cell
}

bool PlannerNode::isBlockedCell(const CellIndex &idx, int gCols, int gRows)
{
  if (idx.x < 0 || idx.x >= gCols || idx.y < 0 || idx.y >= gRows)
  {
    return true;
  }

  int index = static_cast<int>(idx.y * gCols + idx.x);
  // row * columns + column

  if (index < 0 || index >= static_cast<int>(map.data.size()))
  {
    return true;
  }

  return(map.data[index] > 10);
  // TUNE THIS ^^^^
  // TO ADJUST OCCUPANCY VALUES THE ROBOT TREATS AS OBSTACLES
}


double PlannerNode::distance(const CellIndex &a, const CellIndex &b)
{
  // FROM point a to point b
  double dx = a.x - b.x;
  double dy = a.y - b.y;

  return std::sqrt((dx * dx) + (dy * dy));
}

// mfw a*
void PlannerNode::pathPlanner()
{

  pathMessage.poses.clear();
  pathMessage.header.stamp = this->get_clock()->now();
  pathMessage.header.frame_id = "sim_world";
  // map.header.frame_id.empty() ? "map" : map.header.frame_id;

  // get map info
  double gRes = map.info.resolution;
  double gOriginX = map.info.origin.position.x;
  double gOriginY = map.info.origin.position.y;
  int gCols = mapSize;
  int gRows = mapSize;

  int goalCol = static_cast<int>(std::floor((latestGoal.point.x - gOriginX) / gRes));
  int goalRow = static_cast<int>(std::floor((latestGoal.point.y - gOriginY) / gRes));

  // check for out of bounds goal
  if (goalCol < 0 || goalCol >= gCols || goalRow < 0 || goalRow >= gRows)
  {
    return;
  }

  // subtract the relaitve from position from the odometry origin
  // divide by meters/cell to get the "cell" units
  // floor to make it an integer
  int robotCol = static_cast<int>(std::floor((currentPose.pose.pose.position.x - gOriginX) / gRes));
  int robotRow = static_cast<int>(std::floor((currentPose.pose.pose.position.y - gOriginY) / gRes));

  // set robot current position as a* start
  CellIndex start(robotCol, robotRow);

  if (isBlockedCell(start, gCols, gRows))
  {
    return;
  }

  CellIndex goal(goalCol, goalRow);

  // OPEN: vector of all nodes to be evaluated
  std::vector<AStarNode> openSet;

  std::vector<double> gScore(gCols * gRows, INFINITY);
  // initialize a vector of size gCols * gRows ( for all cells ), with initial gScore infinity


  std::vector<int> cameFrom(gCols * gRows, -1);
  // initialized all "parent" indexes stored for each value of cameFrom to -1
  
  // CLOSED vectro of nodes already evaluated
  std::vector<bool> closedSet(gCols * gRows, false);

  int startIndex = static_cast<int>(start.y * gCols + start.x);
  int goalIndex = static_cast<int>(goal.y * gCols + goal.x);
  
  // Y CORRESPONDS WITH ROWS X CORRESPONDS WITH COLUMNS
  // row * columns + column

  gScore[startIndex] = 0.0;

  // add the start node to OPEN
  openSet.push_back(AStarNode(start, start, 0.0, distance(start, goal), distance(start, goal)));
  // AStarNode(CellIndex indx, CellIndex parent, double g, double h, double f)

  const int nCol[8] = { -1, -1, -1,  0,  0,  1,  1,  1};
  const int nRow[8] = { -1,  0,  1, -1,  1, -1,  0,  1};

  // (-1, -1) (-1, 0) (-1, 1)
  // (0,  -1) CURRENT (0,  1)
  // (1,  -1) (1,  0) (1,  1)

  bool foundPath = false;

  while (!openSet.empty())
  {
    // current = node in OPEN with the lowest f cost
    std::size_t bestIndex = 0;
    double bestF = openSet[0].fScore;
    for (std::size_t i = 1; i < openSet.size(); i++)
    {
      if (openSet[i].fScore < bestF)
      {
        bestF = openSet[i].fScore;
        bestIndex = i;
      }
    }

    AStarNode current = openSet[bestIndex];
    openSet.erase(openSet.begin() + static_cast<long>(bestIndex));

    int currentIndex = static_cast<int>(current.index.y * gCols + current.index.x);
    if (currentIndex < 0 || currentIndex >= gCols * gRows)
    {
      continue;
    }

    // remove current from OPEN, add to CLOSED
    if (closedSet[currentIndex])
    {
      continue;
    }
    closedSet[currentIndex] = true;

    // if current is the target node, path has been found
    if (current.index == goal)
    {
      foundPath = true;
      break;
    }

    for (int i = 0; i < 8; i++)
    {
      // foreach neighbour of the current node
      CellIndex neighbor(current.index.x + nCol[i], current.index.y + nRow[i]);
      // if neighbour is not traversable or neighbour is in CLOSED, skip it
      int neighborIndex = static_cast<int>(neighbor.y * gCols + neighbor.x);
      if (neighborIndex < 0 || neighborIndex >= gCols * gRows)
      {
        continue;
      }

      if (isBlockedCell(neighbor, gCols, gRows) || closedSet[neighborIndex])
      {
        continue;
      }

      double stepCost = std::sqrt((nCol[i] * nCol[i]) + (nRow[i] * nRow[i]));
      double tentativeG = gScore[currentIndex] + stepCost;

      // if new path to neighbour is shorter OR neighbour is not in OPEN
      if (tentativeG < gScore[neighborIndex])
      {
        // set parent of neighbour to current
        cameFrom[neighborIndex] = currentIndex;
        // set f_cost of neighbour
        gScore[neighborIndex] = tentativeG;

        double hScore = distance(neighbor, goal);
        double fScore = tentativeG + hScore;
        // if neighbour is not in OPEN, add neighbour to OPEN
        openSet.push_back(AStarNode(neighbor, current.index, tentativeG, hScore, fScore));
      }
    }
  }

  if (!foundPath)
  {
    return;
  }

  // reconstruct and publish
  std::vector<CellIndex> cells;
  int currentIndex = goalIndex;

  while (currentIndex >= 0)
  {
    CellIndex currentCell;
    currentCell.x = currentIndex % gCols;
    currentCell.y = currentIndex / gCols;
    cells.push_back(currentCell);
    currentIndex = cameFrom[currentIndex];
  }

  std::reverse(cells.begin(), cells.end());

  for (std::size_t i = 0; i < cells.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = pathMessage.header.stamp;
    pose.header.frame_id = pathMessage.header.frame_id;

    pose.pose.position.x = gOriginX + (cells[i].x * gRes);
    pose.pose.position.y = gOriginY + (cells[i].y * gRes);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pathMessage.poses.push_back(pose);
  }

  publishedPath->publish(pathMessage);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
