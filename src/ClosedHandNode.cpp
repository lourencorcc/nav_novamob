#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber()
  : Node("simple_subscriber")
  {
    hand_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/hand_topic", 10, std::bind(&SimpleSubscriber::hand_callback, this, std::placeholders::_1));
    
    save_goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/save_goal", 10, std::bind(&SimpleSubscriber::save_goal_callback, this, std::placeholders::_1));

    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  }

  geometry_msgs::msg::PoseStamped previous_goal;
  int control_pub = 0;

private:
  void hand_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if ((msg->data == 1) && (control_pub == 1)) {
      RCLCPP_INFO(this->get_logger(), "Publishing previous goal to /goal_pose");
      goal_publisher_->publish(previous_goal);
      control_pub = 0;
    }
  }

  void save_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Saving goal from /save_goal");
    previous_goal = *msg;
    control_pub = 1;
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hand_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr save_goal_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
