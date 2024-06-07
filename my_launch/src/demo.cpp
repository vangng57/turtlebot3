#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>
#include <iostream>

class RotateAndMoveToGoal : public rclcpp::Node
{
public:
    RotateAndMoveToGoal()
        : Node("rotate_and_move_to_goal"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), state_(State::STOP)
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&RotateAndMoveToGoal::timer_callback, this));
    }

    void set_goal(double x_goal, double y_goal)
    {
        x_goal_ = x_goal;
        y_goal_ = y_goal;
        state_ = State::ROTATE;
    }

private:
    enum class State { ROTATE, MOVE, STOP };
    State state_;

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            
            double current_x = transform_stamped.transform.translation.x;
            double current_y = transform_stamped.transform.translation.y;
            double current_yaw = get_yaw(transform_stamped.transform.rotation);

            double angle_to_goal = std::atan2(y_goal_ - current_y, x_goal_ - current_x);
            double angle_diff = normalize_angle(angle_to_goal - current_yaw);
            double distance = calculate_distance(current_x, current_y, x_goal_, y_goal_);

            auto twist_msg = geometry_msgs::msg::Twist();

            switch (state_)
            {
            case State::ROTATE:
                if (std::abs(angle_diff) > 0.01)
                {
                    twist_msg.angular.z = 1.0 * angle_diff;
                    twist_msg.angular.z = std::min(std::max(twist_msg.angular.z, -1.0), 1.0);
                }
                else
                {
                    twist_msg.angular.z = 0.0;
                    state_ = State::MOVE;
                    std::cout<<"MOVE"<<std::endl;
                }
                break;

            case State::MOVE:
                if (distance > 0.1)
                {
                    std::cout<<"RUN"<<std::endl;
                    std::cout<<"X: "<<distance<<std::endl;
                    //twist_msg.linear.x = 0.5 * distance;
                    //twist_msg.linear.x = std::min(twist_msg.linear.x, 0.22); // Giới hạn vận tốc tuyến tính
                    twist_msg.linear.x = 0.1;
                    std::cout<<"vel: "<<twist_msg.linear.x <<std::endl;
                    twist_msg.angular.z = 0.0;
                }
                else
                {
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    state_ = State::STOP;
                }
                break;

            case State::STOP:
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                break;
            }

            cmd_vel_publisher_->publish(twist_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    double get_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        double roll, pitch, yaw;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double x_goal_;
    double y_goal_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateAndMoveToGoal>();

    double x_goal, y_goal;
    std::cout << "Enter the x-coordinate of the goal: ";
    std::cin >> x_goal;
    std::cout << "Enter the y-coordinate of the goal: ";
    std::cin >> y_goal;

    node->set_goal(x_goal, y_goal);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}