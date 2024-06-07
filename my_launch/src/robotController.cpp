#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <pthread.h>
#include <signal.h>
#include <chrono>
#include <functional>
#include <memory>
#include <math.h>

pid_t main_pid, monitor_pid;

class ControllerMonitor : public rclcpp::Node {
public:
    ControllerMonitor() : rclcpp::Node("controller_monitor") {
        // Subscriber for /plan
        subscription_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&ControllerMonitor::plan_callback, this, std::placeholders::_1));

        // Subscriber for /amcl_pose
        subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&ControllerMonitor::pose_callback, this, std::placeholders::_1));

        // Publisher for /cmd_vel
        publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        publisher_fb = this->create_publisher<std_msgs::msg::String>("/feedback", 10);

        RCLCPP_INFO(this->get_logger(), "Started following the path.");
    }

private:
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // Process the received plan
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Process the received pose
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_fb;
};

void signal_handler(int signum) {
  
    // Handle the signal
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerMonitor>();

    signal(SIGSTOP, signal_handler);

    monitor_pid = fork();
    if (monitor_pid == 0) {
        // Child process code
    } else {
        // Parent process code
        rclcpp::spin(node);
        rclcpp::shutdown();
    }

    return 0;
}