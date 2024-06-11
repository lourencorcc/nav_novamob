#include <string>
#include "../include/nav_novamob/plugins/always_true_condition.hpp"

namespace nav2_behavior_tree
{

AlwaysTrueCondition::AlwaysTrueCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  
}

BT::NodeStatus AlwaysTrueCondition::tick()
{
 
  return BT::NodeStatus::SUCCESS;
 
}

BT::PortsList AlwaysTrueCondition::providedPorts()
{
  return {};
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AlwaysTrueCondition>("AlwaysTrueCondition");
}
