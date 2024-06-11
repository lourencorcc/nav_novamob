#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CLOSED_HAND_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CLOSED_HAND_CONDITION_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ClosedHandCondition that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
class ClosedHandCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ClosedHandCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ClosedHandCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */
  void gestureCallback(std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hand_sub_;
  std::string hand_topic_;
  int gesture_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_