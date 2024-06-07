#include <memory>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_path/msg/custom_path.hpp"


class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        publisher_fb = this->create_publisher<std_msgs::msg::String>("/feedback", 10);
        subscriber_cmd = this->create_subscription<std_msgs::msg::Int32>(
            "cmdToControl", 10,
            std::bind(&PathPublisher::handleCmdToControl, this, std::placeholders::_1));
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&PathPublisher::handleClickedPoint, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Path Publisher has started. Create maximum 30 points for the path.");
    }

    void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
        if (path.poses.size() < sizeOfPath) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position = msg->point;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
            RCLCPP_INFO(this->get_logger(), "Point added to path at x: %f, y: %f", msg->point.x, msg->point.y);
            //publisher_->publish(path);
        } else {
            std::cout << "Maximum number of points reached. Please send the path!" << std::endl;
        }
    }

    void handleCmdToControl(const std_msgs::msg::Int32::SharedPtr msg){
        if(msg->data == 1){
            auto feedback_msg = std_msgs::msg::String();
            std::lock_guard<std::mutex> lock(mutex_);
            if (!path.poses.empty()) {
                publisher_->publish(path);
                RCLCPP_INFO(this->get_logger(), "Final path published with %zu points.", path.poses.size());
                char feedback[100];
                snprintf(feedback, sizeof(feedback), "Final path published with %zu points.", path.poses.size());
                feedback_msg.data = feedback; 
                path.poses.clear();
            } else {
                RCLCPP_INFO(this->get_logger(), "No path to send.");
                feedback_msg.data = "No path to send.";
            }
            publisher_fb->publish(feedback_msg);
        }

    }

    void publishPath() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!path.poses.empty()) {
            publisher_->publish(path);
            RCLCPP_INFO(this->get_logger(), "Final path published with %zu points.", path.poses.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "No path to send.");
        }
    }

private:
    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_fb;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_cmd;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
    std::mutex mutex_;
    const size_t sizeOfPath = 50;
    custom_path::msg::CustomPath points;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
