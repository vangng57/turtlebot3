#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleSubscriberPublisher : public rclcpp::Node
{
public:
  SimpleSubscriberPublisher()
  : Node("simple_sub_pub")
  {
    // Subscriber
    subscription_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&SimpleSubscriberPublisher::amcl_pose_callback, this, std::placeholders::_1));
      
    subscription_plan_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/plan", 10, std::bind(&SimpleSubscriberPublisher::plan_callback, this, std::placeholders::_1));
    
    // Publisher
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback", 10);
    
    // Timer to periodically publish messages
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SimpleSubscriberPublisher::timer_callback, this));
  }

private:
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received AMCL pose: [x: %f, y: %f, z: %f]", 
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void plan_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received plan: [x: %f, y: %f, z: %f]", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }
  
  void timer_callback()
  {
    // Publish a Twist message to /cmd_vel
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.5;
    twist_msg.angular.z = 0.1;
    cmd_vel_publisher_->publish(twist_msg);
    
    // Publish a feedback message to /feedback
    auto feedback_msg = std_msgs::msg::String();
    feedback_msg.data = "Feedback message";
    feedback_publisher_->publish(feedback_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_plan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSubscriberPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}