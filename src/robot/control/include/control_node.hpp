#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // subscribers + publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub;

    // timer for control loop
    rclcpp::TimerBase::SharedPtr timer;

    // stored messages
    nav_msgs::msg::Path currentPath;
    nav_msgs::msg::Odometry currentOdom;
    bool hasPath;
    bool hasOdom;

    // values
    double lookaheadDistance;
    double goalTolerance;
    double linearSpeed;
    double angularKp;

    // callbacks + helpers
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool findLookaheadPoint(geometry_msgs::msg::PoseStamped &lookahead);
    bool atGoal();
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
    double distance2d(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
};

#endif
