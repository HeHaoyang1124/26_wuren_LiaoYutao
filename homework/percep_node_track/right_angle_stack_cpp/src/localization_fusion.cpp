// 定位融合节点：GPS + IMU陀螺 + 轮速计 + 磁力计 → 融合位姿
// GPS 做低通滤波融合位置，IMU/轮速计做死推算，磁力计修正航向角

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "right_angle_stack_cpp/utils.hpp"

using namespace right_angle_stack_cpp;

static constexpr double EARTH_RADIUS_M = 6378137.0;

class LocalizationFusion : public rclcpp::Node
{
public:
  LocalizationFusion()
  : Node("localization_fusion")
  {
    // 声明参数
    this->declare_parameter("origin_latitude", 23.043055);
    this->declare_parameter("origin_longitude", 113.397222);
    this->declare_parameter("initial_x", 0.0);
    this->declare_parameter("initial_y", -15.0);
    this->declare_parameter("initial_yaw", M_PI / 2.0);
    this->declare_parameter("gps_gain", 0.12);            // GPS 位置融合的平滑系数
    this->declare_parameter("mag_gain", 0.0);             // 磁力计航向修正系数
    this->declare_parameter("magnetic_declination", 0.0);
    this->declare_parameter("use_first_gps_as_origin", true);
    this->declare_parameter("gps_reject_distance", 8.0);   // GPS 跳变拒绝阈值
    this->declare_parameter("mag_reject_angle", 1.2);      // 磁力计跳变拒绝阈值
    this->declare_parameter("gps_topic", "/sensors/gps/fix");
    this->declare_parameter("imu_topic", "/sensors/imu/data_raw");
    this->declare_parameter("wheel_odom_topic", "/sensors/wheel_odom");
    this->declare_parameter("magnetic_field_topic", "/sensors/magnetic_field");

    // 读取参数
    origin_lat_ = this->get_parameter("origin_latitude").as_double() * M_PI / 180.0;
    origin_lon_ = this->get_parameter("origin_longitude").as_double() * M_PI / 180.0;
    gps_gain_ = this->get_parameter("gps_gain").as_double();
    mag_gain_ = this->get_parameter("mag_gain").as_double();
    magnetic_declination_ = this->get_parameter("magnetic_declination").as_double();
    use_first_gps_as_origin_ = this->get_parameter("use_first_gps_as_origin").as_bool();
    gps_reject_distance_ = this->get_parameter("gps_reject_distance").as_double();
    mag_reject_angle_ = this->get_parameter("mag_reject_angle").as_double();
    initial_x_ = this->get_parameter("initial_x").as_double();
    initial_y_ = this->get_parameter("initial_y").as_double();

    // 初始化状态
    x_ = initial_x_;
    y_ = initial_y_;
    yaw_ = this->get_parameter("initial_yaw").as_double();
    gps_ref_lat_ = origin_lat_;
    gps_ref_lon_ = origin_lon_;
    gps_reference_ready_ = !use_first_gps_as_origin_;

    // 发布器
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 订阅传感器
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->get_parameter("gps_topic").as_string(), 10,
      std::bind(&LocalizationFusion::on_gps, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      this->get_parameter("imu_topic").as_string(), 50,
      std::bind(&LocalizationFusion::on_imu, this, std::placeholders::_1));
    wheel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("wheel_odom_topic").as_string(), 20,
      std::bind(&LocalizationFusion::on_wheel_odom, this, std::placeholders::_1));
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      this->get_parameter("magnetic_field_topic").as_string(), 20,
      std::bind(&LocalizationFusion::on_magnetic_field, this, std::placeholders::_1));

    // 定时器：每 20ms 做一次死推算 + 发布位姿
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
      std::bind(&LocalizationFusion::on_timer, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Localization fusion started: GPS + wheel odom + IMU gyro + magnetometer heading.");
  }

private:
  // GPS 经纬度转本地 ENU 坐标
  void gps_to_local_xy(double latitude_deg, double longitude_deg, double & x, double & y)
  {
    double lat = latitude_deg * M_PI / 180.0;
    double lon = longitude_deg * M_PI / 180.0;
    x = initial_x_ + EARTH_RADIUS_M * std::cos(gps_ref_lat_) * (lon - gps_ref_lon_);
    y = initial_y_ + EARTH_RADIUS_M * (lat - gps_ref_lat_);
  }

  // GPS 回调：低通滤波融合位置，带跳变拒绝
  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;

    // 首次 GPS 作为参考原点
    if (!gps_reference_ready_) {
      gps_ref_lat_ = msg->latitude * M_PI / 180.0;
      gps_ref_lon_ = msg->longitude * M_PI / 180.0;
      gps_reference_ready_ = true;
      RCLCPP_INFO(this->get_logger(),
        "GPS reference initialized from first fix; first fix maps to initial pose (%.2f, %.2f).",
        initial_x_, initial_y_);
    }

    double gps_x, gps_y;
    gps_to_local_xy(msg->latitude, msg->longitude, gps_x, gps_y);

    // 跳变拒绝：GPS 位置与当前融合位姿差太远则丢弃
    if (std::hypot(gps_x - x_, gps_y - y_) > gps_reject_distance_) {
      rejected_gps_count_++;
      if (rejected_gps_count_ == 1 || rejected_gps_count_ == 20 || rejected_gps_count_ == 100) {
        RCLCPP_WARN(this->get_logger(),
          "Rejected GPS fix far from fused pose: gps=(%.2f, %.2f), pose=(%.2f, %.2f).",
          gps_x, gps_y, x_, y_);
      }
      return;
    }

    // 低通滤波：平滑更新位置
    x_ = (1.0 - gps_gain_) * x_ + gps_gain_ * gps_x;
    y_ = (1.0 - gps_gain_) * y_ + gps_gain_ * gps_y;
  }

  // IMU 回调：记录角速度
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    yaw_rate_ = msg->angular_velocity.z;
  }

  // 轮速计回调：记录前进速度，低速时用轮速计角速度补充
  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    forward_speed_ = msg->twist.twist.linear.x;
    if (std::abs(yaw_rate_) < 1e-4) {
      yaw_rate_ = msg->twist.twist.angular.z;
    }
  }

  // 磁力计回调：修正航向角，带跳变拒绝
  void on_magnetic_field(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    if (mag_gain_ <= 0.0) return;
    double mx = msg->magnetic_field.x;
    double my = msg->magnetic_field.y;
    if (std::abs(mx) + std::abs(my) < 1e-9) return;
    double measured_yaw = std::atan2(mx, my) - magnetic_declination_;
    double yaw_error = normalize_angle(measured_yaw - yaw_);

    // 跳变拒绝
    if (std::abs(yaw_error) > mag_reject_angle_) {
      rejected_mag_count_++;
      if (rejected_mag_count_ == 1 || rejected_mag_count_ == 20 || rejected_mag_count_ == 100) {
        RCLCPP_WARN(this->get_logger(),
          "Rejected magnetometer heading jump: measured=%.3f, fused=%.3f, error=%.3f.",
          measured_yaw, yaw_, yaw_error);
      }
      return;
    }
    yaw_ += mag_gain_ * yaw_error;
    yaw_ = normalize_angle(yaw_);
  }

  // 定时器回调：死推算（航位推算） + 发布位姿
  void on_timer()
  {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.2) dt = 0.02;

    // 死推算：航向 + 角速度积分，位置 + 速度积分
    yaw_ = normalize_angle(yaw_ + yaw_rate_ * dt);
    x_ += forward_speed_ * std::cos(yaw_) * dt;
    y_ += forward_speed_ * std::sin(yaw_) * dt;

    publish_state(now);
  }

  // 发布位姿、里程计和 TF 变换
  void publish_state(rclcpp::Time stamp)
  {
    auto quat = yaw_to_quaternion(yaw_);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "world";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation = quat;
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose.pose;
    odom.twist.twist.linear.x = forward_speed_;
    odom.twist.twist.angular.z = yaw_rate_;
    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[35] = 0.05;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = quat;
    tf_broadcaster_->sendTransform(transform);
  }

  double origin_lat_, origin_lon_;
  double gps_gain_, mag_gain_, magnetic_declination_;
  bool use_first_gps_as_origin_;
  double gps_reject_distance_, mag_reject_angle_;
  double initial_x_, initial_y_;
  double x_, y_, yaw_;
  double forward_speed_ = 0.0;
  double yaw_rate_ = 0.0;
  double gps_ref_lat_, gps_ref_lon_;
  bool gps_reference_ready_ = false;
  int rejected_gps_count_ = 0;
  int rejected_mag_count_ = 0;
  rclcpp::Time last_time_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationFusion>());
  rclcpp::shutdown();
  return 0;
}