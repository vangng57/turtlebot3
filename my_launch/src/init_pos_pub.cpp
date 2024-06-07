#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher")
    {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&InitialPosePublisher::odom_callback, this, std::placeholders::_1));

        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Set a timer to shut down after 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(7),
            std::bind(&InitialPosePublisher::shutdown_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose_msg.header.stamp = this->now();
        initial_pose_msg.header.frame_id = "map";  // or "odom" if you prefer

        initial_pose_msg.pose.pose = msg->pose.pose;
        
        // Optionally, you can set the covariance if needed
        for (size_t i = 0; i < 36; ++i)
        {
            initial_pose_msg.pose.covariance[i] = msg->pose.covariance[i];
        }

        initial_pose_publisher_->publish(initial_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose");
    }

    void shutdown_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down initial_pose_publisher node.");
        rclcpp::shutdown();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}