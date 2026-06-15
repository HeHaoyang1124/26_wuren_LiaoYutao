import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from tf2_ros import TransformBroadcaster

from .utils import normalize_angle, yaw_to_quaternion

"""定位融合节点。

该节点负责把 Gazebo 中的多个传感器数据组合成车辆在 `world` 坐标系下的位姿。

输入：

- `/sensors/gps/fix`：GPS 经纬度。
- `/sensors/imu/data_raw`：IMU 角速度，主要使用 z 轴 yaw rate。
- `/sensors/wheel_odom`：轮速里程计，主要使用车体前向速度。
- `/sensors/magnetic_field`：磁力计，保留航向修正能力，但默认关闭强融合。

输出：

- `/localization/pose`：`geometry_msgs/msg/PoseStamped`，给感知和建图使用。
- `/localization/odom`：`nav_msgs/msg/Odometry`，给 Pure Pursuit 控制器使用。
- TF `world -> base_link`：给 RViz 和其他 ROS 工具使用。

实现不是完整 EKF，而是一个针对本仿真任务的轻量互补融合：

- GPS 提供慢速位置修正；
- 轮速里程计和 IMU 在两帧之间做短时积分；
- 磁力计接口保留，但默认 `mag_gain=0.0`，防止未标定时航向跳变影响闭环。
"""


EARTH_RADIUS_M = 6378137.0


class LocalizationFusion(Node):
    def __init__(self):
        super().__init__('localization_fusion')

        # 经纬度原点。world 文件中也定义了 spherical_coordinates。
        # 实际调试中发现 Gazebo NavSat 输出和固定原点可能存在偏差，
        # 所以下面还提供 use_first_gps_as_origin 作为仿真参考点对齐方案。
        self.declare_parameter('origin_latitude', 23.043055)
        self.declare_parameter('origin_longitude', 113.397222)

        # 车辆初始位姿。课程要求起点为 (0, -15)，朝北，因此 yaw = pi/2。
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', -15.0)
        self.declare_parameter('initial_yaw', math.pi / 2.0)

        # 互补融合增益。gps_gain 越大，GPS 对位置修正越强；
        # mag_gain 越大，磁力计对航向修正越强。当前默认配置中 mag_gain=0。
        self.declare_parameter('gps_gain', 0.45)
        self.declare_parameter('mag_gain', 0.30)
        self.declare_parameter('magnetic_declination', 0.0)

        # 调试中 GPS 曾把位姿拉到几万米外。
        # use_first_gps_as_origin=True 表示第一帧 GPS 不直接当绝对坐标，
        # 而是把第一帧映射到 initial_x/initial_y，后续只使用相对位移。
        self.declare_parameter('use_first_gps_as_origin', True)

        # 传感器异常保护：GPS 或磁力计跳变过大时拒绝该次观测。
        self.declare_parameter('gps_reject_distance', 8.0)
        self.declare_parameter('mag_reject_angle', 1.2)

        # 传感器话题名做成参数，便于更换仿真后端或外部感知包时复用节点。
        self.declare_parameter('gps_topic', '/sensors/gps/fix')
        self.declare_parameter('imu_topic', '/sensors/imu/data_raw')
        self.declare_parameter('wheel_odom_topic', '/sensors/wheel_odom')
        self.declare_parameter('magnetic_field_topic', '/sensors/magnetic_field')

        self.origin_lat = math.radians(self.get_parameter('origin_latitude').value)
        self.origin_lon = math.radians(self.get_parameter('origin_longitude').value)
        self.gps_gain = float(self.get_parameter('gps_gain').value)
        self.mag_gain = float(self.get_parameter('mag_gain').value)
        self.magnetic_declination = float(self.get_parameter('magnetic_declination').value)
        self.use_first_gps_as_origin = bool(self.get_parameter('use_first_gps_as_origin').value)
        self.gps_reject_distance = float(self.get_parameter('gps_reject_distance').value)
        self.mag_reject_angle = float(self.get_parameter('mag_reject_angle').value)

        # 融合状态初始化。x/y/yaw 始终表示车辆 base_link 在 world 中的位姿。
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.forward_speed = 0.0
        self.yaw_rate = 0.0
        self.last_time = self.get_clock().now()

        # GPS 参考经纬度。若 use_first_gps_as_origin=True，则在第一帧 GPS 到来时覆盖。
        self.gps_ref_lat = self.origin_lat
        self.gps_ref_lon = self.origin_lon
        self.gps_reference_ready = not self.use_first_gps_as_origin

        # 只在特定次数打印拒绝日志，避免传感器持续异常时刷屏。
        self.rejected_gps_count = 0
        self.rejected_mag_count = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odom', 10)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(
            NavSatFix,
            self.get_parameter('gps_topic').value,
            self.on_gps,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.on_imu,
            50,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('wheel_odom_topic').value,
            self.on_wheel_odom,
            20,
        )
        self.create_subscription(
            MagneticField,
            self.get_parameter('magnetic_field_topic').value,
            self.on_magnetic_field,
            20,
        )

        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info('Localization fusion started: GPS + wheel odom + IMU gyro + magnetometer heading.')

    def gps_to_local_xy(self, latitude_deg, longitude_deg):
        """把 GPS 经纬度转换成 world 平面米制坐标。

        使用局部切平面近似：

        - x = R * cos(lat0) * delta_lon
        - y = R * delta_lat

        这里再叠加 initial_x/initial_y，使第一帧 GPS 可以对应课程要求的起点。
        """
        lat = math.radians(latitude_deg)
        lon = math.radians(longitude_deg)
        x = self.initial_x + EARTH_RADIUS_M * math.cos(self.gps_ref_lat) * (lon - self.gps_ref_lon)
        y = self.initial_y + EARTH_RADIUS_M * (lat - self.gps_ref_lat)
        return x, y

    def on_gps(self, msg):
        """处理 GPS 观测。

        GPS 主要用于限制里程计积分漂移，但仿真中如果参考经纬度不一致，会产生巨大跳变。
        因此先做第一帧参考点初始化，再做距离门限拒绝。
        """
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        # 第一帧 GPS 映射到初始位姿，避免 NavSat 绝对原点不一致导致位置飞走。
        if not self.gps_reference_ready:
            self.gps_ref_lat = math.radians(msg.latitude)
            self.gps_ref_lon = math.radians(msg.longitude)
            self.gps_reference_ready = True
            self.get_logger().info(
                'GPS reference initialized from first fix; first fix maps to initial pose '
                f'({self.initial_x:.2f}, {self.initial_y:.2f}).'
            )
        gps_x, gps_y = self.gps_to_local_xy(msg.latitude, msg.longitude)

        # 如果 GPS 和当前融合位姿差距过大，认为是异常观测。
        # 直角弯任务速度较低，单帧跳 8 m 基本不合理。
        if math.hypot(gps_x - self.x, gps_y - self.y) > self.gps_reject_distance:
            self.rejected_gps_count += 1
            if self.rejected_gps_count in (1, 20, 100):
                self.get_logger().warn(
                    'Rejected GPS fix far from fused pose: '
                    f'gps=({gps_x:.2f}, {gps_y:.2f}), pose=({self.x:.2f}, {self.y:.2f}).'
                )
            return

        # 简单互补融合：当前位置向 GPS 位置缓慢靠拢。
        self.x = (1.0 - self.gps_gain) * self.x + self.gps_gain * gps_x
        self.y = (1.0 - self.gps_gain) * self.y + self.gps_gain * gps_y

    def on_imu(self, msg):
        """读取 IMU z 轴角速度作为航向积分输入。"""
        self.yaw_rate = msg.angular_velocity.z

    def on_wheel_odom(self, msg):
        """读取车体前向速度。

        DiffDrive 发布的 wheel odom 中 linear.x 表示车辆前向速度。
        如果 IMU 暂时没有有效 yaw_rate，则用 odom 自带角速度兜底。
        """
        self.forward_speed = msg.twist.twist.linear.x
        if abs(self.yaw_rate) < 1e-4:
            self.yaw_rate = msg.twist.twist.angular.z

    def on_magnetic_field(self, msg):
        """处理磁力计航向观测。

        当前配置默认 `mag_gain=0.0`，因此不会进入后续融合。
        保留这段代码是为了后续标定磁力计坐标系和磁偏角后能重新启用。
        """
        if self.mag_gain <= 0.0:
            return
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        if abs(mx) + abs(my) < 1e-9:
            return
        measured_yaw = math.atan2(mx, my) - self.magnetic_declination
        yaw_error = normalize_angle(measured_yaw - self.yaw)

        # 未标定磁力计时，航向可能突然跳变。跳变过大就拒绝，防止控制链路被带偏。
        if abs(yaw_error) > self.mag_reject_angle:
            self.rejected_mag_count += 1
            if self.rejected_mag_count in (1, 20, 100):
                self.get_logger().warn(
                    'Rejected magnetometer heading jump: '
                    f'measured={measured_yaw:.3f}, fused={self.yaw:.3f}, error={yaw_error:.3f}.'
                )
            return
        self.yaw += self.mag_gain * yaw_error
        self.yaw = normalize_angle(self.yaw)

    def on_timer(self):
        """固定频率预测并发布融合状态。

        这里用轮速和 yaw_rate 做短时航迹推算：

        - yaw += yaw_rate * dt
        - x += v * cos(yaw) * dt
        - y += v * sin(yaw) * dt

        GPS 回调会异步修正 x/y，磁力计回调会异步修正 yaw。
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 仿真刚启动或 /clock 跳变时 dt 可能异常，限制到一个合理默认值。
        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        self.yaw = normalize_angle(self.yaw + self.yaw_rate * dt)
        self.x += self.forward_speed * math.cos(self.yaw) * dt
        self.y += self.forward_speed * math.sin(self.yaw) * dt

        self.publish_state(now)

    def publish_state(self, stamp):
        """发布 Pose、Odometry 和 TF。

        三者表达的是同一个融合位姿，只是面向不同消费者：

        - PoseStamped：感知、建图等上层算法；
        - Odometry：控制器需要速度字段；
        - TF：RViz 和 ROS 工具使用。
        """
        quat = yaw_to_quaternion(self.yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = quat
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = self.forward_speed
        odom.twist.twist.angular.z = self.yaw_rate
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quat
        self.tf_pub.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
