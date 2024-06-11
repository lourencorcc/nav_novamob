#include "../include/nav_novamob/plugins/resume_nav_node.hpp"

namespace nav2_behavior_tree
{

ResumeNavNode::ResumeNavNode(
    const std::string& name, 
    const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Create action client for NavigateToPose
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList ResumeNavNode::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("previous_goal"),
    };
}

BT::NodeStatus ResumeNavNode::tick() 
{
    RCLCPP_INFO(node_->get_logger(), "Resuming navigation with previous goal...");

    geometry_msgs::msg::PoseStamped previous_goal;
    if (!getInput("previous_goal", previous_goal))
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [previous_goal].");
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = previous_goal;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) { /* handle result */ };

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Previous goal has been sent successfully.");
    return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ResumeNavNode>("ResumeNavNode");
}
