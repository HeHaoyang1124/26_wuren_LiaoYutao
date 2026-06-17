// 直角转弯路径规划器：从锥桶地图生成中心线路径，或使用解析几何的硬编码路径

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "right_angle_stack_cpp/utils.hpp"

using namespace right_angle_stack_cpp;

class RightAnglePlanner : public rclcpp::Node
{
public:
  RightAnglePlanner()
  : Node("right_angle_planner")
  {
    this->declare_parameter("prefer_cone_map", true);
    this->declare_parameter("fallback_path", true);
    this->declare_parameter("turn_center_x", 12.0);    // 转弯圆心 X
    this->declare_parameter("turn_center_y", 0.0);     // 转弯圆心 Y
    this->declare_parameter("turn_radius", 12.0);      // 转弯半径
    this->declare_parameter("pair_distance_max", 6.2); // 蓝黄锥桶配对最大距离
    this->declare_parameter("path_step", 0.8);         // 路径点间距

    prefer_cone_map_ = this->get_parameter("prefer_cone_map").as_bool();
    use_fallback_ = this->get_parameter("fallback_path").as_bool();
    center_x_ = this->get_parameter("turn_center_x").as_double();
    center_y_ = this->get_parameter("turn_center_y").as_double();
    radius_ = this->get_parameter("turn_radius").as_double();
    pair_distance_max_ = this->get_parameter("pair_distance_max").as_double();
    path_step_ = this->get_parameter("path_step").as_double();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/centerline", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/planning", 10);
    map_sub_ = this->create_subscription<fsd_common_msgs::msg::Map>(
      "/estimation/slam/map", 10,
      std::bind(&RightAnglePlanner::on_map, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose", 10,
      std::bind(&RightAnglePlanner::on_pose, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RightAnglePlanner::on_timer, this));
    RCLCPP_INFO(this->get_logger(), "Right-angle centerline planner started.");
  }

private:
  void on_map(const fsd_common_msgs::msg::Map::SharedPtr msg) { latest_map_ = msg; }
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = msg; }

  // 计算路径点在赛道上的累计进度（用于排序中点）
  // 直道段用 y 坐标，弯道段用弧长
  double track_progress(const std::pair<double, double> & point)
  {
    double x = point.first, y = point.second;
    if (y <= 0.2 && x < 3.0) {
      return y + 15.0;  // 起始直道
    }
    double theta = std::atan2(y - center_y_, x - center_x_);
    if (x <= center_x_ + 1.0 && y >= -0.5) {
      theta = std::min(M_PI, std::max(M_PI / 2.0, theta));
      return 15.0 + (M_PI - theta) * radius_;  // 弯道弧长
    }
    return 15.0 + 0.5 * M_PI * radius_ + std::max(0.0, x - center_x_);  // 终点直道
  }

  // 解析路径：直行 → 四分之一圆弧 → 直行
  std::vector<std::pair<double, double>> analytic_path()
  {
    std::vector<std::pair<double, double>> points;
    // 起始直道（y 从 -15 到 0，x=0）
    double y = -15.0;
    while (y <= 0.0) {
      points.emplace_back(0.0, y);
      y += path_step_;
    }
    // 转弯（以 center 为圆心，半径 radius，从 π 到 π/2）
    double theta = M_PI;
    double theta_end = M_PI / 2.0;
    double dtheta = path_step_ / std::max(radius_, 0.1);
    while (theta >= theta_end) {
      double x = center_x_ + radius_ * std::cos(theta);
      double yp = center_y_ + radius_ * std::sin(theta);
      points.emplace_back(x, yp);
      theta -= dtheta;
    }
    // 终点直道（x 从 center_x 到 32）
    double x = center_x_;
    while (x <= 32.0) {
      points.emplace_back(x, center_y_ + radius_);
      x += path_step_;
    }
    return points;
  }

  // 锥桶中心线：配对蓝/黄锥桶 → 取中点 → 按赛道进度排序 → 加密点
  std::vector<std::pair<double, double>> cone_centerline()
  {
    if (!latest_map_) return {};
    std::vector<std::pair<double, double>> blue, yellow;
    for (const auto & c : latest_map_->cone_blue) blue.emplace_back(c.position.x, c.position.y);
    for (const auto & c : latest_map_->cone_yellow) yellow.emplace_back(c.position.x, c.position.y);
    if (blue.size() < 3 || yellow.size() < 3) return {};

    std::vector<bool> used_yellow(yellow.size(), false);
    std::vector<std::pair<double, double>> midpoints;

    // 贪心配对：每个蓝锥桶找最近的未配对黄锥桶
    for (const auto & [bx, by] : blue) {
      int best_index = -1;
      double best_dist = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < yellow.size(); ++i) {
        if (used_yellow[i]) continue;
        double dist = std::hypot(bx - yellow[i].first, by - yellow[i].second);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = static_cast<int>(i);
        }
      }
      if (best_index < 0 || best_dist > pair_distance_max_) continue;
      used_yellow[best_index] = true;
      auto [yx, yy] = yellow[best_index];
      midpoints.emplace_back((bx + yx) * 0.5, (by + yy) * 0.5);
    }

    // 按赛道进度排序，保证路径点顺序正确
    std::sort(midpoints.begin(), midpoints.end(),
      [this](const auto & a, const auto & b) { return track_progress(a) < track_progress(b); });

    if (midpoints.size() < 5) return {};
    return densify(midpoints);
  }

  // 在相邻点之间插值加密，使路径点间距 ≤ path_step_
  std::vector<std::pair<double, double>> densify(const std::vector<std::pair<double, double>> & points)
  {
    if (points.empty()) return {};
    std::vector<std::pair<double, double>> dense = {points[0]};
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      auto [sx, sy] = points[i];
      auto [ex, ey] = points[i + 1];
      double dist = std::hypot(ex - sx, ey - sy);
      int steps = std::max(1, static_cast<int>(dist / path_step_));
      for (int j = 1; j <= steps; ++j) {
        double ratio = static_cast<double>(j) / steps;
        dense.emplace_back(sx + ratio * (ex - sx), sy + ratio * (ey - sy));
      }
    }
    return dense;
  }

  // 选择路径：优先锥桶中心线，不足则回退到解析路径
  std::pair<std::vector<std::pair<double, double>>, std::string> choose_path()
  {
    auto cone_path = prefer_cone_map_ ? cone_centerline() : std::vector<std::pair<double, double>>{};
    if (cone_path.size() >= 8) return {cone_path, "cone_map"};
    if (use_fallback_) return {analytic_path(), "analytic"};
    return {{}, "none"};
  }

  void on_timer()
  {
    auto [points, source] = choose_path();
    if (points.empty()) return;

    auto stamp = this->now();
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "world";

    for (size_t i = 0; i < points.size(); ++i) {
      auto [x, y] = points[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;

      // 计算路径点朝向（沿路径切线方向）
      double yaw = 0.0;
      if (i + 1 < points.size()) {
        yaw = std::atan2(points[i + 1].second - y, points[i + 1].first - x);
      } else if (i > 0) {
        yaw = std::atan2(y - points[i - 1].second, x - points[i - 1].first);
      }
      pose.pose.orientation = yaw_to_quaternion(yaw);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    publish_markers(path.header, points, source);
  }

  // 发布 RViz 可视化：锥桶中心线为绿色，解析路径为白色
  void publish_markers(const std_msgs::msg::Header & header,
                       const std::vector<std::pair<double, double>> & points,
                       const std::string & source)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker line;
    line.header = header;
    line.ns = "centerline";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.12;
    line.color.a = 1.0;

    if (source == "cone_map") {
      line.color.r = 0.0;
      line.color.g = 0.9;
      line.color.b = 0.35;
    } else {
      line.color.r = 1.0;
      line.color.g = 1.0;
      line.color.b = 1.0;
    }

    for (const auto & [x, y] : points) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = 0.05;
      line.points.push_back(p);
    }
    markers.markers.push_back(line);
    marker_pub_->publish(markers);
  }

  bool prefer_cone_map_, use_fallback_;
  double center_x_, center_y_, radius_;
  double pair_distance_max_, path_step_;
  fsd_common_msgs::msg::Map::SharedPtr latest_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightAnglePlanner>());
  rclcpp::shutdown();
  return 0;
}