#ifndef MY_COSTMAP_PLUGIN_HPP_
#define MY_COSTMAP_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace costmap_plugin
{

class WorkAreaLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  WorkAreaLayer();
  virtual ~WorkAreaLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  virtual void activate() override;
  virtual void deactivate() override;
  virtual void reset() override;
  virtual bool isClearable() override;

private:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message);
  void updateLineCosts(nav2_costmap_2d::Costmap2D& master_grid, const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_subscriber_;
  std::vector<geometry_msgs::msg::Point> work_area_points_;
  double min_x_, min_y_, max_x_, max_y_;
};

}  // namespace costmap_plugin

#endif  // MY_COSTMAP_PLUGIN_HPP_