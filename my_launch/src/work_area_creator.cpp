#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

class WorkAreaCreator : public rclcpp::Node
{
public:
    WorkAreaCreator() : Node("work_area_creator")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10, std::bind(&WorkAreaCreator::point_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("work_area", 10);
    }

private:
    std::vector<geometry_msgs::msg::Point> points_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        points_.push_back(msg->point);
        // Ví dụ: Giả sử điểm cuối được đăng ký khi nhận phím Enter hoặc thông qua giao diện người dùng
        if (msg->header.frame_id == "end") {
            if (check_intersections()) {
                publish_area();
            } else {
                RCLCPP_INFO(this->get_logger(), "Intersecting lines detected. Not publishing the area.");
            }
            points_.clear();
        }
    }

    bool check_intersections()
    {
        // Kiểm tra giao nhau của các đoạn thẳng tạo bởi các điểm
        // Thực hiện thuật toán kiểm tra giao nhau ở đây
        return true; // Placeholder
    }

    void publish_area()
    {
        geometry_msgs::msg::PolygonStamped polygon_msg;
        polygon_msg.header.stamp = this->get_clock()->now();
        polygon_msg.header.frame_id = "map"; // Thay đổi frame_id phù hợp với bối cảnh của bạn
        for (auto& point : points_) {
            geometry_msgs::msg::Point32 point32;
            point32.x = point.x;
            point32.y = point.y;
            point32.z = point.z;
            polygon_msg.polygon.points.push_back(point32);
        }
        publisher_->publish(polygon_msg);
        RCLCPP_INFO(this->get_logger(), "Published the work area.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorkAreaCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
