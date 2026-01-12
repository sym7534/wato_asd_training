#include <chrono>
#include <memory>
#include <functional> // for std::bind
#include <cmath> // for sin + cos
 
#include "costmap_node.hpp"
 
// rewrote given code to make the costmap mode publish nav_msgs
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // make the node publisher part
  publishedCostmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));

  //subscription
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::topicCallback, this, std::placeholders::_1));
}

void CostmapNode::initializeCostmap()
{
  // initialize 2d array 
  for(int i = 0; i < SIZE; i++)
  {
    for(int j = 0; j < SIZE; j++)
    {
      costmap[i][j] = 0;
    }
  }
}


// Define the timer to publish a message every 500ms

void CostmapNode::publishCostmap() 
{
  nav_msgs::msg::OccupancyGrid message; // makes a message
  message.header.stamp = this->get_clock()->now(); // gets timestamp for costmap
  message.header.frame_id = "robot/chassis/lidar"; //global coordinate map

  message.info.origin.position.x = origin_x;
  message.info.origin.position.y = origin_y;
  message.info.origin.position.z = 0.0;

  message.info.origin.orientation.w = 1.0; // conveys proper orientation of the origin

  message.info.width = SIZE;
  message.info.height = SIZE;
  message.info.resolution = resolution; // idk what to set this to
  message.data.resize(SIZE*SIZE);

  for (int i = 0; i < SIZE; i++) // i = row, j = col
  {
    for (int j = 0; j < SIZE; j++)
    {
      int index = i*SIZE + j;
      message.data[index] =  static_cast<int8_t>(costmap[i][j]);
    }
  }

  publishedCostmap_->publish(message);
}
 
// topicCallback.. or whatever.. stole the name from some random guide on simple ros subscribers
void CostmapNode::topicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  initializeCostmap();

  // LIDAR DATA

  for(size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double range = msg->ranges[i];

    if(range < msg->range_min || range > msg->range_max)
    {
      continue;
    }

    double angle = (msg->angle_min) + (i)*(msg->angle_increment);

    double x = (range * std::cos(angle));
    double y = (range * std::sin(angle));

    const double origin_x = - (SIZE * resolution) / 2.0;
    const double origin_y = - (SIZE * resolution) / 2.0;

    int mapCol = static_cast<int>(std::floor((x - origin_x) / resolution));
    int mapRow = static_cast<int>(std::floor((y - origin_y) / resolution));

    if (mapRow >= 0 && mapRow < SIZE && mapCol >= 0 && mapCol < SIZE)
    {
      costmap[mapRow][mapCol] = 100.0; // arbitrary "occupied" value
    }
  }

  inflateCostmap();

  publishCostmap();
}

// inflate costmap function
void CostmapNode::inflateCostmap()
{
  // filler
  const int radius_cells = std::ceil(inflation_radius / resolution);

  double inflatedCostmap[SIZE][SIZE];
  for(int i = 0; i < SIZE; i++)
  {
    for(int j = 0; j < SIZE; j++)
    {
      inflatedCostmap[i][j] = costmap[i][j];
    }
  }

  for(int i = 0; i < SIZE; i++)
  {
    for(int j = 0; j < SIZE; j++)
    {
      if(costmap[i][j] < 100.0)
      {
        continue; // cell does NOT contain an obstacle
      }

      for(int dRow = -radius_cells; dRow <= radius_cells; dRow++)
      {
        for(int dCol = -radius_cells; dCol <= radius_cells; dCol++)
        {
          int row = i + dRow; // i represents rows, j represents cols
          int col = j + dCol;

          if(row < 0 || row >= SIZE || col < 0 || col >= SIZE)
          {
            continue;
          }

          double distanceToCell = std::sqrt((dRow * resolution) * (dRow * resolution) +
                                            (dCol * resolution) * (dCol * resolution));
          if (distanceToCell > inflation_radius)
          {
            continue;
          }

          double cellCost = max_cost * (1.0 - (distanceToCell / inflation_radius));
          if(cellCost > inflatedCostmap[row][col])
          {
            inflatedCostmap[row][col] = cellCost;
          }
        }
      }
    }
  }

  for(int i = 0; i < SIZE; i++)
  {
    for(int j = 0; j< SIZE; j++)
    {
      costmap[i][j] = inflatedCostmap[i][j];
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();


  return 0;
}
