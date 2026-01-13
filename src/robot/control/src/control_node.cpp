#include "control_node.hpp"

#include <chrono>

ControlNode::ControlNode() : Node("control")
{
  // default values
  hasPath = false;
  hasOdom = false;

  lookaheadDistance = 1;
  goalTolerance = 0.4;
  linearSpeed = 1.5;
  angularKp = 2.0;

  // subscriptions
  pathSub = this->create_subscription<nav_msgs::msg::Path>("/path",10,std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odomSub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered",10,std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // publisher
  cmdPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // timer for control loop
  timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  currentPath = *msg;
  hasPath = !currentPath.poses.empty();
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  currentOdom = *msg;
  hasOdom = true;
}

void ControlNode::timerCallback()
{
  if (!hasPath || !hasOdom)
  {
    return;
  }

  if (atGoal())
  {
    geometry_msgs::msg::Twist stop;
    cmdPub->publish(stop);
    return;
  }

  geometry_msgs::msg::PoseStamped lookahead;
  bool found = findLookaheadPoint(lookahead);
  if (!found)
  {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  double robotYaw = getYawFromQuaternion(currentOdom.pose.pose.orientation);
  double dx = lookahead.pose.position.x - currentOdom.pose.pose.position.x;
  double dy = lookahead.pose.position.y - currentOdom.pose.pose.position.y;
  double targetAngle = std::atan2(dy, dx);

  // keep angle between -pi and pi
  double headingError = targetAngle - robotYaw;
  while (headingError > M_PI)
  {
    headingError -= (2.0 * M_PI);
  }
  while (headingError < -M_PI)
  {
    headingError += (2.0 * M_PI);
  }

  cmd.linear.x = linearSpeed;
  cmd.angular.z = angularKp * headingError;

  cmdPub->publish(cmd);
}

bool ControlNode::findLookaheadPoint(geometry_msgs::msg::PoseStamped &lookahead)
{
  if (currentPath.poses.empty())
  {
    return false;
  }

  geometry_msgs::msg::Point robotPoint = currentOdom.pose.pose.position;

  for (std::size_t i = 0; i < currentPath.poses.size(); i++)
  {
    double dist = distance2d(robotPoint, currentPath.poses[i].pose.position);
    if (dist >= lookaheadDistance)
    {
      lookahead = currentPath.poses[i];
      return true;
    }
  }

  // if no lookahead, just use the last point on the path
  lookahead = currentPath.poses[currentPath.poses.size() - 1];
  return true;
}

bool ControlNode::atGoal()
{
  if (currentPath.poses.empty())
  {
    return true;
  }

  geometry_msgs::msg::Point robotPoint = currentOdom.pose.pose.position;
  geometry_msgs::msg::Point goalPoint = currentPath.poses[currentPath.poses.size() - 1].pose.position;
  double distToGoal = distance2d(robotPoint, goalPoint);

  return(distToGoal <= goalTolerance);
}

double ControlNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

double ControlNode::distance2d(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
