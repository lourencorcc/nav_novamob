#include <string>

#include "../include/nav_novamob/plugins/closed_hand_condition.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_behavior_tree
{

ClosedHandCondition::ClosedHandCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  hand_topic_("/hand_topic"),
  gesture_(-1) // should be -1 by default
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  hand_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    hand_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ClosedHandCondition::gestureCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus ClosedHandCondition::tick()
{
  RCLCPP_INFO(node_->get_logger(), "INSIDE CLOSED HAND CONDITION NODE TICK!");
  
  callback_group_executor_.spin_some();
  if (gesture_ == 1) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void ClosedHandCondition::gestureCallback(std_msgs::msg::Int32::SharedPtr msg)
{
    gesture_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClosedHandCondition>("ClosedHandCondition");
}