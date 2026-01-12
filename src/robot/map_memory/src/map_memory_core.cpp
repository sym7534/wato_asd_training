#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore()
{
}

void MapMemoryCore::initializeGlobalMap(double resolution, int cols, int rows, double originX, double originY)
{
  // give the globalmap message data
  globalMap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  // map size
  globalMap->info.resolution = resolution;
  globalMap->info.width = cols;
  globalMap->info.height = rows;

  // arbitrary origin location + orientation
  globalMap->info.origin.position.x = originX;
  globalMap->info.origin.position.y = originY;
  globalMap->info.origin.position.z = 0.0;
  globalMap->info.origin.orientation.w = 1.0;

  globalMap->header.frame_id = "sim_world"; // testing 
  // FIXED FRAMEGAJPREJGESJ

  // start with everything unknown
  globalMap->data.resize(cols * rows);
  for (int row = 0; row < rows; row++)
  {
    for (int col = 0; col < cols; col++)
    {
      const int index = (row * cols) + col; // NOT A 2D ARRAY
      globalMap->data[index] = -1; 
    }
  }
}

void MapMemoryCore::updateGlobalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr localCostmap,
  double robotX,
  double robotY,
  double robotYaw)
{
  // check if the global map exists, or if there is a current costmap
  if (!globalMap || !localCostmap)
  {
    return;
  }

  // local map data
  double lRes = localCostmap->info.resolution;
  double lOriginX = localCostmap->info.origin.position.x;
  double lOriginY = localCostmap->info.origin.position.y;
  int lCols = localCostmap->info.width;
  int lRows = localCostmap->info.height;

  // global map data
  double gRes = globalMap->info.resolution;
  double gOriginX = globalMap->info.origin.position.x;
  double gOriginY = globalMap->info.origin.position.y;
  int gCols = globalMap->info.width;
  int gRows = globalMap->info.height;

  // rotation matrix terms for the robot pose
  double cosYaw = std::cos(robotYaw);
  double sinYaw = std::sin(robotYaw);

  for (int row = 0; row < lRows; row++)
  {
    for (int col = 0; col < lCols; col++)
    {
      //get the costmap value from the cell at the local costmap
      int index = static_cast<int>(row * lCols + col);
      int8_t cellValue = localCostmap->data[index];

      // calculating the local position from local costmap
      double lX = lOriginX + (static_cast<double>(col)) * lRes;
      double lY = lOriginY + (static_cast<double>(row)) * lRes;

      // accounting for difference in robot rotation between local gostmap and global costmap
      double worldX = robotX + (lX * cosYaw - lY * sinYaw);
      double worldY = robotY + (lX * sinYaw + lY * cosYaw);

      int gCol = static_cast<int>(std::floor((worldX - gOriginX) / gRes));
      int gRow = static_cast<int>(std::floor((worldY - gOriginY) / gRes));

      if (gRow < 0 || gRow >= gRows || gCol < 0 || gCol >= gCols)
      {
        continue;
      }

      // overwrite because local data is known
      globalMap->data[gRow * gCols + gCol] = cellValue;
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap()
{
  // just pass the shared pointer back up to the node
  return globalMap;
}

} 
