#include "MyCostmapPlugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

namespace costmap_plugin
{

WorkAreaLayer::WorkAreaLayer() : min_x_(0), min_y_(0), max_x_(0), max_y_(0) {}

WorkAreaLayer::~WorkAreaLayer() {}

void WorkAreaLayer::onInitialize()
{
    current_ = true;
    default_value_ = nav2_costmap_2d::NO_INFORMATION;

    if (auto node = node_.lock())
    {
        polygon_subscriber_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/work_area", rclcpp::SystemDefaultsQoS(),
            std::bind(&WorkAreaLayer::polygonCallback, this, std::placeholders::_1));
    }
    else
    {
        throw std::runtime_error{"Failed to lock node"};
    }

    matchSize();
}

void WorkAreaLayer::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
{
    work_area_points_.clear();
    for (const auto& p : message->polygon.points) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        work_area_points_.push_back(point);
        min_x_ = std::min(min_x_, point.x);
        max_x_ = std::max(max_x_, point.x);
        min_y_ = std::min(min_y_, point.y);
        max_y_ = std::max(max_y_, point.y);
    }
}

void WorkAreaLayer::updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x = std::min(*min_x, this->min_x_);
    *min_y = std::min(*min_y, this->min_y_);
    *max_x = std::max(*max_x, this->max_x_);
    *max_y = std::max(*max_y, this->max_y_);
}

void WorkAreaLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
    if (work_area_points_.empty()) return;

    for (size_t i = 0; i < work_area_points_.size() - 1; ++i)
    {
        updateLineCosts(master_grid, work_area_points_[i], work_area_points_[i+1]);
    }
}

void WorkAreaLayer::updateLineCosts(nav2_costmap_2d::Costmap2D& master_grid, const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end)
{
    int x0, y0, x1, y1;
    master_grid.worldToMapNoBounds(start.x, start.y, x0, y0);
    master_grid.worldToMapNoBounds(end.x, end.y, x1, y1);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        master_grid.setCost(x0, y0, nav2_costmap_2d::LETHAL_OBSTACLE);

        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void WorkAreaLayer::deactivate()
{
    polygon_subscriber_ = nullptr;
}

void WorkAreaLayer::activate()
{
    if (auto node = node_.lock())
    {
        polygon_subscriber_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/work_area", rclcpp::SystemDefaultsQoS(),
            std::bind(&WorkAreaLayer::polygonCallback, this, std::placeholders::_1));
    }
}

void WorkAreaLayer::reset()
{
    deactivate();
    activate();
}

bool WorkAreaLayer::isClearable()
{
    return true;
}

}  // namespace costmap_plugin

PLUGINLIB_EXPORT_CLASS(costmap_plugin::WorkAreaLayer, nav2_costmap_2d::Layer)