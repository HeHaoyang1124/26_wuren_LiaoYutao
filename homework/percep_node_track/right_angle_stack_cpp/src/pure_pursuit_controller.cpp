// 纯跟踪控制器：在路径上找前视距离处的目标点，计算曲率→横摆角速度，弯道自动减速

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "right_angle_stack_cpp/utils.hpp"

using namespace right_angle_stack_cpp;

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
  : Node("pure_pursuit_controller")
  {
    this->declare_parameter("target_speed", 3.0);        // 目标速度 (m/s)
    this->declare_parameter("min_speed", 1.2);            // 弯道最低速度
    this->declare_parameter("lookahead_distance", 4.0);   // 前视距离
    this->declare_parameter("max_yaw_rate", 1.4);         // 最大横摆角速度
    this->declare_parameter("stop_distance", 1.2);        // 停车距离阈值

    target_speed_ = this->get_parameter("target_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    stop_distance_ = this->get_parameter("stop_distance").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planning/centerline", 10,
      std::bind(&PurePursuitController::on_path, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", 20,
      std::bind(&PurePursuitController::on_odom, this, std::placeholders::_1));

    // 50ms 控制周期
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&PurePursuitController::on_timer, this));
    RCLCPP_INFO(this->get_logger(), "Pure pursuit controller started.");
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_.clear();
    for (const auto & pose : msg->poses) {
      path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    state_x_ = msg->pose.pose.position.x;
    state_y_ = msg->pose.pose.position.y;
    state_yaw_ = quaternion_to_yaw(msg->pose.pose.orientation);
    has_state_ = true;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void on_timer()
  {
    if (!has_state_ || path_.size() < 2) {
      publish_stop();
      return;
    }

    double x = state_x_, y = state_y_, yaw = state_yaw_;

    // 找最近路径点
    size_t nearest_index = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.size(); ++i) {
      double dist = std::hypot(path_[i].first - x, path_[i].second - y);
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest_index = i;
      }
    }

    // 接近终点时停车
    auto [goal_x, goal_y] = path_.back();
    if (nearest_index >= path_.size() - 4 &&
        std::hypot(goal_x - x, goal_y - y) < stop_distance_) {
      publish_stop();
      return;
    }

    // 纯跟踪：从最近点往后找，找到第一个距车辆 ≥ lookahead_distance 且在前方的点
    auto target = path_.back();
    for (size_t i = nearest_index; i < path_.size(); ++i) {
      double dx = path_[i].first - x;
      double dy = path_[i].second - y;
      double local_x, local_y;
      world_to_body(dx, dy, yaw, local_x, local_y);
      if (local_x > 0.2 && std::hypot(dx, dy) >= lookahead_distance_) {
        target = path_[i];
        break;
      }
    }

    // 计算目标点在车体坐标系下的位置
    double local_x, local_y;
    world_to_body(target.first - x, target.second - y, yaw, local_x, local_y);
    double lookahead = std::max(0.5, std::hypot(local_x, local_y));

    // 纯跟踪公式：曲率 = 2 * lateral_error / lookahead²
    double curvature = 2.0 * local_y / (lookahead * lookahead);
    double yaw_error = std::atan2(local_y, std::max(local_x, 1e-3));

    // 横摆角速度 = 速度 × 曲率 + 航向误差补偿
    double yaw_rate = target_speed_ * curvature;
    yaw_rate += 0.20 * normalize_angle(yaw_error);
    yaw_rate = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, yaw_rate));

    // 弯道减速：曲率越大，速度越低
    double turn_slowdown = std::min(1.0, std::abs(curvature) / 0.28);
    double speed = target_speed_ * (1.0 - 0.45 * turn_slowdown);
    speed = std::max(min_speed_, speed);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = yaw_rate;
    cmd_pub_->publish(cmd);
  }

  double target_speed_, min_speed_, lookahead_distance_, max_yaw_rate_, stop_distance_;
  std::vector<std::pair<double, double>> path_;
  double state_x_ = 0.0, state_y_ = 0.0, state_yaw_ = 0.0;
  bool has_state_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}