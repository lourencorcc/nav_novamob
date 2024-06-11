#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ALWAYS_TRUE_CONDITION_HPP
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ALWAYS_TRUE_CONDITION_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/condition_node.h>


namespace nav2_behavior_tree
{

class AlwaysTrueCondition : public BT::ConditionNode
{
public:
  AlwaysTrueCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}
#endif // ALWAYS_TRUE_CONDITION_HPP
