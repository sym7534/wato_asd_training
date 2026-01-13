#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    void initializePath();

  private:
    rclcpp::Logger logger_;
};

}  

#endif  
