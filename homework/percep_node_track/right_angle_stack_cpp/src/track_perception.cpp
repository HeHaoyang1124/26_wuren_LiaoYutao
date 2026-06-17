// 赛道感知节点：模拟感知模块，从 SDF 加载锥桶世界坐标
// 根据车辆位置将视野内锥桶转换到 base_link 坐标系，加入噪声模拟真实感知

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/cone.hpp>
#include <fsd_common_msgs/msg/cone_detections.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "right_angle_stack_cpp/utils.hpp"
#include "right_angle_stack_cpp/track_model.hpp"

using namespace right_angle_stack_cpp;

// 默认锥桶列表（SDF 加载失败时使用）
static const std::vector<ConeInfo> DEFAULT_CONES = {
  {"blue", -2.0, -15.0, 0.0}, {"yellow", 2.0, -15.0, 0.0},
  {"blue", -2.0, -10.0, 0.0}, {"yellow", 2.0, -10.0, 0.0},
  {"blue", -2.0, -5.0, 0.0}, {"yellow", 2.0, -5.0, 0.0},
  {"blue", -2.0, 0.0, 0.0}, {"yellow", 2.0, 0.0, 0.0},
  {"blue", -1.31, 4.32, 0.0}, {"yellow", 2.76, 3.83, 0.0},
  {"blue", 0.68, 8.23, 0.0}, {"yellow", 4.93, 7.07, 0.0},
  {"blue", 3.77, 11.32, 0.0}, {"yellow", 8.17, 9.24, 0.0},
  {"blue", 7.68, 13.31, 0.0}, {"yellow", 12.0, 10.0, 0.0},
  {"blue", 12.0, 14.0, 0.0}, {"yellow", 17.0, 10.0, 0.0},
  {"blue", 17.0, 14.0, 0.0}, {"yellow", 22.0, 10.0, 0.0},
  {"blue", 22.0, 14.0, 0.0}, {"yellow", 27.0, 10.0, 0.0},
  {"blue", 27.0, 14.0, 0.0},
};

class TrackPerception : public rclcpp::Node
{
public:
  TrackPerception()
  : Node("track_perception"), gen_(rd_())
  {
    this->declare_parameter("track_sdf", "");
    this->declare_parameter("max_range", 18.0);       // 前向感知最大距离
    this->declare_parameter("lateral_range", 9.0);    // 侧向感知最大距离
    this->declare_parameter("rear_margin", 1.0);      // 后方负距离（车后盲区）
    this->declare_parameter("position_noise_std", 0.05); // 位置噪声标准差
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("map_topic", "/perception/cones");
    this->declare_parameter("detections_topic", "/perception/cone_detections");

    max_range_ = this->get_parameter("max_range").as_double();
    lateral_range_ = this->get_parameter("lateral_range").as_double();
    rear_margin_ = this->get_parameter("rear_margin").as_double();
    noise_std_ = this->get_parameter("position_noise_std").as_double();

    // 从 SDF 加载锥桶，失败则使用内置默认列表
    std::string track_sdf = this->get_parameter("track_sdf").as_string();
    if (!track_sdf.empty()) {
      try {
        cones_ = load_cones_from_sdf(track_sdf);
      } catch (const std::exception & exc) {
        RCLCPP_WARN(this->get_logger(), "Failed to load track SDF, using built-in cone list: %s", exc.what());
        cones_ = DEFAULT_CONES;
      }
    } else {
      cones_ = DEFAULT_CONES;
    }

    x_ = 0.0;
    y_ = -15.0;
    yaw_ = 1.57079632679;

    map_pub_ = this->create_publisher<fsd_common_msgs::msg::Map>(
      this->get_parameter("map_topic").as_string(), 10);
    det_pub_ = this->create_publisher<fsd_common_msgs::msg::ConeDetections>(
      this->get_parameter("detections_topic").as_string(), 10);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose", 10,
      std::bind(&TrackPerception::on_pose, this, std::placeholders::_1));

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&TrackPerception::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
      "Track perception publishing %zu known cones in base_link frame.", cones_.size());
  }

private:
  // 接收定位位姿
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    x_ = msg->pose.position.x;
    y_ = msg->pose.position.y;
    yaw_ = quaternion_to_yaw(msg->pose.orientation);
  }

  // 构造一个带噪声的局部锥桶
  fsd_common_msgs::msg::Cone make_local_cone(const std::string & color, double local_x, double local_y, double z)
  {
    std::normal_distribution<double> noise(0.0, noise_std_);
    fsd_common_msgs::msg::Cone cone;
    cone.position.x = local_x + noise(gen_);
    cone.position.y = local_y + noise(gen_);
    cone.position.z = z;
    cone.color = color;
    cone.pose_confidence = 0.9;
    cone.color_confidence = 0.98;
    return cone;
  }

  // 定时发布：将视野内锥桶从世界坐标转换到车体坐标
  void on_timer()
  {
    auto stamp = this->now();
    fsd_common_msgs::msg::Map cone_map;
    cone_map.header.stamp = stamp;
    cone_map.header.frame_id = "base_link";

    fsd_common_msgs::msg::ConeDetections detections;
    detections.header = cone_map.header;

    for (const auto & ci : cones_) {
      // 世界坐标 → 车体坐标
      double local_x, local_y;
      world_to_body(ci.x - x_, ci.y - y_, yaw_, local_x, local_y);

      // 视野裁剪：前方、侧向、后方盲区
      if (local_x < -rear_margin_ || local_x > max_range_) continue;
      if (std::abs(local_y) > lateral_range_) continue;

      auto cone = make_local_cone(ci.color, local_x, local_y, ci.z);
      detections.cone_detections.push_back(cone);

      // 按颜色归类
      if (ci.color == "blue") {
        cone_map.cone_blue.push_back(cone);
      } else if (ci.color == "yellow") {
        cone_map.cone_yellow.push_back(cone);
      } else if (ci.color == "red") {
        cone_map.cone_red.push_back(cone);
      } else {
        cone_map.cone_unknown.push_back(cone);
      }
    }

    map_pub_->publish(cone_map);
    det_pub_->publish(detections);
  }

  std::vector<ConeInfo> cones_;
  double x_, y_, yaw_;
  double max_range_, lateral_range_, rear_margin_, noise_std_;
  std::random_device rd_;
  std::mt19937 gen_;
  rclcpp::Publisher<fsd_common_msgs::msg::Map>::SharedPtr map_pub_;
  rclcpp::Publisher<fsd_common_msgs::msg::ConeDetections>::SharedPtr det_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackPerception>());
  rclcpp::shutdown();
  return 0;
}