#include "nav_novamob/plugins/stop_robot_action.hpp"
#include "behaviortree_cpp_v3/decorators/retry_node.h"

namespace nav2_behavior_tree
{
StopRobotNode::StopRobotNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    goal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    
    current_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 
        rclcpp::SystemDefaultsQoS(), 
        std::bind(&StopRobotNode::currentPositionCallback, this, std::placeholders::_1),
        sub_option);
}

BT::PortsList StopRobotNode::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("current_goal"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("previous_goal"),
        // BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
}

BT::NodeStatus StopRobotNode::tick() 
{
    
    RCLCPP_INFO(node_->get_logger(), "Stopping the robot...");

    geometry_msgs::msg::PoseStamped current_goal;
    if (!getInput("current_goal", current_goal))
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [goal].");
        return BT::NodeStatus::FAILURE;
    }

    // Save the current goal to the previous_goal port
    setOutput("previous_goal", current_goal);
    RCLCPP_INFO(node_->get_logger(), "Previous goal: x: %f, y: %f, z: %f",
                    current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z);
    
    // Use the last known position as the new current goal
    auto new_goal = geometry_msgs::msg::PoseStamped();
    new_goal.header.stamp = rclcpp::Clock().now();
    new_goal.header.frame_id = "map";
    callback_group_executor_.spin_some();
    new_goal.pose = current_position_;

    goal_pose_pub_->publish(new_goal);
    // setOutput("goal", new_goal);

    // Publish zero velocity to stop the robot
    // auto stop_msg = geometry_msgs::msg::Twist();
    // cmd_vel_pub_->publish(stop_msg);

    return BT::NodeStatus::SUCCESS;
}

// void StopRobotNode::halt()
// {
//     RCLCPP_INFO(node_->get_logger(), "StopRobotNode halted.");
// }

void StopRobotNode::currentPositionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_position_ = msg->pose.pose;
    RCLCPP_INFO(node_->get_logger(), "Current position: x: %f, y: %f, z: %f",
                    current_position_.position.x, current_position_.position.y, current_position_.position.z);

    // If you want to print orientation as well:
    RCLCPP_INFO(node_->get_logger(), "Current orientation: x: %f, y: %f, z: %f, w: %f",
                current_position_.orientation.x, current_position_.orientation.y,
                current_position_.orientation.z, current_position_.orientation.w);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::StopRobotNode>("StopRobotNode");
    factory.registerNodeType<BT::RetryNode>("Retry");
}