#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ComparePoseNode : public rclcpp::Node
{
public:
    ComparePoseNode() : Node("compare_pose_node"), last_odom_time_(this->now()), last_amcl_time_(this->now())
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ComparePoseNode::odomCallback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&ComparePoseNode::amclCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto now = this->now();
        if ((now - last_odom_time_).seconds() >= 0.5)
        {
            last_odom_time_ = now;
            auto position = msg->pose.pose.position;
            auto orientation = msg->pose.pose.orientation;
            double roll, pitch, yaw;
            tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "Odom Position: x=%.2f, y=%.2f, yaw=%.2f", position.x, position.y, yaw);
        }
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        auto now = this->now();
        if ((now - last_amcl_time_).seconds() >= 0.5)
        {
            last_amcl_time_ = now;
            auto position = msg->pose.pose.position;
            auto orientation = msg->pose.pose.orientation;
            double roll, pitch, yaw;
            tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "AMCL Position: x=%.2f, y=%.2f, yaw=%.2f", position.x, position.y, yaw);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    rclcpp::Time last_odom_time_;
    rclcpp::Time last_amcl_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComparePoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}