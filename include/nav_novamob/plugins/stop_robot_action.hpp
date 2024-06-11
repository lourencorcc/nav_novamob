#ifndef STOP_ROBOT_ACTION_HPP
#define STOP_ROBOT_ACTION_HPP

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace nav2_behavior_tree
{
class StopRobotNode : public BT::SyncActionNode
{
public:
    StopRobotNode(const std::string & name, const BT::NodeConfiguration & config);
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    // void halt() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_position_sub_;
    geometry_msgs::msg::Pose current_position_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    void currentPositionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
}  // namespace nav2_behavior_tree

#endif  // STOP_ROBOT_ACTION_HPP