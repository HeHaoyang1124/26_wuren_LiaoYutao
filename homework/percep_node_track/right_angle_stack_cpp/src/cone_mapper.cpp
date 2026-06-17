// 锥桶建图节点：将局部感知锥桶变换到世界坐标系，合并邻近锥桶，发布全局地图

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/cone.hpp>
#include <fsd_common_msgs/msg/cone_detections.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "right_angle_stack_cpp/utils.hpp"

using namespace right_angle_stack_cpp;

// 路标：世界坐标 + 被观测次数 + 置信度
struct Landmark
{
  double x, y;
  int count;
  double pose_confidence;
  double color_confidence;
};

class ConeMapper : public rclcpp::Node
{
public:
  ConeMapper()
  : Node("cone_mapper")
  {
    this->declare_parameter("perception_map_topic", "/perception/cones");
    this->declare_parameter("perception_detections_topic", "/perception/cone_detections");
    this->declare_parameter("merge_distance", 0.75);   // 锥桶合并距离阈值
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("base_frame", "base_link");

    merge_distance_ = this->get_parameter("merge_distance").as_double();
    world_frame_ = this->get_parameter("world_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    map_pub_ = this->create_publisher<fsd_common_msgs::msg::Map>("/estimation/slam/map", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/cone_map", 10);

    // 订阅定位位姿，用于坐标变换
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose", 10,
      std::bind(&ConeMapper::on_pose, this, std::placeholders::_1));

    // 订阅感知结果（Map 格式）
    map_sub_ = this->create_subscription<fsd_common_msgs::msg::Map>(
      this->get_parameter("perception_map_topic").as_string(), 10,
      std::bind(&ConeMapper::on_map_detection, this, std::placeholders::_1));

    // 订阅感知结果（ConeDetections 格式）
    det_sub_ = this->create_subscription<fsd_common_msgs::msg::ConeDetections>(
      this->get_parameter("perception_detections_topic").as_string(), 10,
      std::bind(&ConeMapper::on_cone_detections, this, std::placeholders::_1));

    // 定时发布地图和可视化
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
      std::bind(&ConeMapper::publish_outputs, this));

    RCLCPP_INFO(this->get_logger(),
      "Cone mapper started. Local perception is transformed into the world ENU frame.");
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_yaw_ = quaternion_to_yaw(msg->pose.orientation);
    has_pose_ = true;
  }

  void on_map_detection(const fsd_common_msgs::msg::Map::SharedPtr msg)
  {
    process_cones(msg->header.frame_id, msg->cone_blue, "blue");
    process_cones(msg->header.frame_id, msg->cone_yellow, "yellow");
    process_cones(msg->header.frame_id, msg->cone_red, "red");
    process_cones(msg->header.frame_id, msg->cone_unknown, "unknown");
    publish_outputs();
  }

  void on_cone_detections(const fsd_common_msgs::msg::ConeDetections::SharedPtr msg)
  {
    process_cones(msg->header.frame_id, msg->cone_detections, "");
    publish_outputs();
  }

  // 处理一组锥桶：坐标变换 → 合并
  void process_cones(
    const std::string & frame_id,
    const std::vector<fsd_common_msgs::msg::Cone> & cones,
    const std::string & fallback_color)
  {
    for (const auto & cone : cones) {
      std::string color = cone.color.empty() ? fallback_color : cone.color;
      std::string key = color_key(color);
      double wx, wy;
      if (!to_world(frame_id, cone, wx, wy)) continue;
      merge_landmark(key, wx, wy, cone.pose_confidence, cone.color_confidence);
    }
  }

  // 将锥桶坐标转换到世界坐标系
  bool to_world(const std::string & frame_id, const fsd_common_msgs::msg::Cone & cone, double & wx, double & wy)
  {
    std::string frame = frame_id;
    if (!frame.empty() && frame[0] == '/') frame = frame.substr(1);
    if (frame == world_frame_ || frame == "map") {
      wx = cone.position.x;
      wy = cone.position.y;
      return true;
    }
    if (!has_pose_) return false;
    body_to_world(cone.position.x, cone.position.y, pose_x_, pose_y_, pose_yaw_, wx, wy);
    return true;
  }

  // 合并路标：最近邻匹配，距离小于阈值则加权平均更新，否则新增
  void merge_landmark(const std::string & color, double x, double y,
                      double pose_confidence, double color_confidence)
  {
    auto & bucket = landmarks_[color];
    Landmark * closest = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();

    for (auto & lm : bucket) {
      double dist = std::hypot(lm.x - x, lm.y - y);
      if (dist < best_distance) {
        best_distance = dist;
        closest = &lm;
      }
    }

    if (closest == nullptr || best_distance > merge_distance_) {
      // 新增路标
      bucket.push_back({x, y, 1, pose_confidence, color_confidence});
      return;
    }

    // 加权平均更新已有路标，count 上限 30
    int count = std::min(closest->count + 1, 30);
    double alpha = 1.0 / count;
    closest->x = (1.0 - alpha) * closest->x + alpha * x;
    closest->y = (1.0 - alpha) * closest->y + alpha * y;
    closest->count = count;
    closest->pose_confidence = std::max(closest->pose_confidence, pose_confidence);
    closest->color_confidence = std::max(closest->color_confidence, color_confidence);
  }

  fsd_common_msgs::msg::Cone make_cone(const std::string & color, const Landmark & lm)
  {
    fsd_common_msgs::msg::Cone cone;
    cone.position.x = lm.x;
    cone.position.y = lm.y;
    cone.position.z = 0.0;
    cone.color = color;
    cone.pose_confidence = lm.pose_confidence;
    cone.color_confidence = lm.color_confidence;
    return cone;
  }

  // 发布全局地图 + RViz 可视化
  void publish_outputs()
  {
    auto stamp = this->now();
    fsd_common_msgs::msg::Map msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;

    for (const auto & lm : landmarks_["blue"]) msg.cone_blue.push_back(make_cone("blue", lm));
    for (const auto & lm : landmarks_["yellow"]) msg.cone_yellow.push_back(make_cone("yellow", lm));
    for (const auto & lm : landmarks_["red"]) msg.cone_red.push_back(make_cone("red", lm));
    for (const auto & lm : landmarks_["unknown"]) msg.cone_unknown.push_back(make_cone("unknown", lm));
    map_pub_->publish(msg);

    // RViz Marker 可视化：每个锥桶画一个圆柱体
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = msg.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const auto & [color, bucket] : landmarks_) {
      for (const auto & lm : bucket) {
        visualization_msgs::msg::Marker marker;
        marker.header = msg.header;
        marker.ns = color + "_cones";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = lm.x;
        marker.pose.position.y = lm.y;
        marker.pose.position.z = 0.28;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.35;
        marker.scale.y = 0.35;
        marker.scale.z = 0.56;
        set_marker_color(marker, color);
        markers.markers.push_back(marker);
      }
    }
    marker_pub_->publish(markers);
  }

  double merge_distance_;
  std::string world_frame_, base_frame_;
  double pose_x_ = 0.0, pose_y_ = 0.0, pose_yaw_ = 0.0;
  bool has_pose_ = false;
  std::map<std::string, std::vector<Landmark>> landmarks_;

  rclcpp::Publisher<fsd_common_msgs::msg::Map>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
  rclcpp::Subscription<fsd_common_msgs::msg::ConeDetections>::SharedPtr det_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMapper>());
  rclcpp::shutdown();
  return 0;
}