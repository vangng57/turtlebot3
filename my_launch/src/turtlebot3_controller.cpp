#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

class PathController : public rclcpp::Node
{
public:
    PathController(pid_t child_pid)
    : Node("path_controller"), child_pid_(child_pid)
    {
        // Subscription to plan
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10, std::bind(&PathController::path_callback, this, std::placeholders::_1));

        // Subscription to amcl_pose
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "amcl_pose", 10, std::bind(&PathController::pose_callback, this, std::placeholders::_1));

        // Publish for /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if(update_path){
            // clear the path
            path_.poses.clear();
            // receive the path
            path_.poses = msg->poses;
            RCLCPP_INFO(this->get_logger(), "Received path");
            if(path_.poses.empty()){
                RCLCPP_INFO(this->get_logger(), "The Path is empty.");
                return;
            }
            // After receiving path, subscribe to amcl_pose
            RCLCPP_INFO(this->get_logger(), "Press 'n' or 'N' to start at the nearest pose");
            RCLCPP_INFO(this->get_logger(), "Press 'f' or 'F' to start at the first pose");
            RCLCPP_INFO(this->get_logger(), "Press 'x' or 'X' to delete the path");
            char str;
            while(std::cin>>str){
                if(str == 'n' || str == 'N'){
                    pathFollower(findTheShortestWay());
                    break;
                }else if(str == 'f' || str == 'F'){
                    pathFollower(0);
                    break;
                }else if(str == 'x' || str == 'X'){
                    break;
                }else{
                    std::cout<<"WRONG CMD !!!"<<std::endl; 
                }
            }
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose at (%.2f, %.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
        // Perform control logic here
        current_pose.pose.pose.position.x = msg->pose.pose.position.x;
        current_pose.pose.pose.position.y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_angle);
        if (current_angle > M_PI) current_angle -= 2 * M_PI;
        if (current_angle < -M_PI) current_angle += 2 * M_PI;
    }

    void pathFollower(int index){
        while(index <= path_.poses.size()){
            controller(path_.poses[index]);
            index++;
        }
    }

    size_t findTheShortestWay(){
        size_t shortest_index = 0;
        double* distance = new double[path_.poses.size()-1];
        double shortest_distance;

        // calculate distances
        for(size_t i = 0;i < (path_.poses.size()-1);i++){
            double x_err = path_.poses[i+1].pose.position.x - path_.poses[i].pose.position.x;
            double y_err = path_.poses[i+1].pose.position.y - path_.poses[i].pose.position.y;
            double numerator = x_err*(path_.poses[i].pose.position.y - current_pose.pose.pose.position.y) - (path_.poses[i].pose.position.x -current_pose.pose.pose.position.x)*y_err;
            distance[i] = abs(numerator)/sqrt(x_err*x_err + y_err*y_err);
            if(i == 0){
                shortest_distance = distance[i];
            }else{
                if(distance[i] <= shortest_distance){
                    shortest_distance = distance[i];
                    shortest_index = i;
                } 
            }
        }

        //find the nearest pose on the path
        double x1 = path_.poses[shortest_index].pose.position.x;
        double y1 = path_.poses[shortest_index].pose.position.y;
        double x2 = path_.poses[shortest_index + 1].pose.position.x;
        double y2 = path_.poses[shortest_index + 1].pose.position.y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = ((current_pose.pose.pose.position.x - x1) * dx + (current_pose.pose.pose.position.y - y1) * dy) / (dx * dx + dy * dy);
        geometry_msgs::msg::PoseStamped nearest_pose;
        nearest_pose.pose.position.x = x1 + t * dx;
        nearest_pose.pose.position.y = y1 + t * dy;
        controller(nearest_pose);

        delete[] distance;
        return shortest_index+1;
    }

    void controller(geometry_msgs::msg::PoseStamped target_pose){
        kill(child_pid_, SIGUSR1);
        bool cond = true;
        while(rclcpp::ok()){
            // update data
            rclcpp::spin_some(shared_from_this());
            // controller
            if(error_angle(target_pose)>0.01 && (cond == true)){
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = kp_theta_ * error_angle(target_pose);
                publisher_->publish(cmd_vel);
            }else{
                cond = false;
            }

            if(error_position(target_pose)>0.01 && (cond == false)){
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = kp_x_ * error_position(target_pose);
                cmd_vel.angular.z = kp_theta_ * error_angle(target_pose);
                publisher_->publish(cmd_vel);
            }

            if(error_position(target_pose)<=0.01 && (cond == false)){
                break;
            }

        }
    }

    double error_angle(geometry_msgs::msg::PoseStamped target_pose){
        double target_angle = atan2(target_pose.pose.position.y - current_pose.pose.pose.position.y,target_pose.pose.position.x - current_pose.pose.pose.position.x);
        return abs(target_angle - current_angle);
    }

    double error_position(geometry_msgs::msg::PoseStamped target_pose){
        double x_err = current_pose.pose.pose.position.x - target_pose.pose.position.x;
        double y_err = current_pose.pose.pose.position.y - target_pose.pose.position.y;
        return sqrt(x_err*x_err + y_err*y_err);;
    }


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
    double current_angle;
    pid_t child_pid_;
    bool update_path = true;
    double kp_theta_ = 3;
    double kp_x_ = 3;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    pid_t child_pid = fork();
    if(child_pid == 0){
        sleep(1);
        std::cout<<"PRESS 'p' to PAUSE, 'c' to contunue, 's' to TERMINATE after receive the path data."<<std::endl;
        pause();
        char str;
        while(std::cin>>str){
            if((str == 'p') || (str == 'P')){
                std::cout<<"PAUSE THE PROCESS"<<std::endl;
                if (kill(getppid(), SIGSTOP) == -1) {
                perror("pause");
                }
            }else if((str == 'c') || (str == 'C')){
                std::cout<<"CONTINUE FOLOWING THE PATH. "<<std::endl;
                if(kill(getppid(),SIGCONT) == -1) {
                perror("continue");
                }
            }else if((str == 's') || (str == 'S')){
                std::cout<<"STOP FOLLOW THE PATH !!!"<<std::endl;
                if (kill(getppid(), SIGINT) == -1) {
                perror("kill");
                }
                break;
            }else{
            std::cout<<"WRONG CMD !!!"<<std::endl;
            }
        }
    }else{
        auto node = std::make_shared<PathController>(child_pid);
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    return 0;
}