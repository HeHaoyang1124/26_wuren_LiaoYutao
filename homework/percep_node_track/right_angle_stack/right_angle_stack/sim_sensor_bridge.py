import math
import random

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus

from .utils import quaternion_to_yaw, world_to_body, yaw_to_quaternion

"""Gazebo Classic 传感器兜底桥。

这个节点主要服务于早期 Gazebo Classic 路线或传感器插件缺失的场景。
当前主线是 Gazebo Sim 8，通常不需要启用它。

它从 `/gazebo/model_states` 读取车辆真实状态，然后人工生成：

- GPS；
- IMU；
- 磁力计；
- 可选 wheel odom。

这样即使 Gazebo 中某些传感器插件不可用，也能先验证定位、建图、规划、控制链路。
"""


EARTH_RADIUS_M = 6378137.0


class SimSensorBridge(Node):
    """Optional fallback for systems missing Gazebo GPS or magnetometer plugins."""

    def __init__(self):
        super().__init__('sim_sensor_bridge')

        # Gazebo Classic 的 /gazebo/model_states 中包含所有模型，需要指定车辆模型名。
        self.declare_parameter('model_name', 'racecar')

        # 经纬度原点，用来把 world 米制坐标转换成 GPS 经纬度。
        self.declare_parameter('origin_latitude', 23.043055)
        self.declare_parameter('origin_longitude', 113.397222)

        # 给合成传感器加入少量噪声，让下游融合逻辑更接近真实传感器。
        self.declare_parameter('gps_noise_std', 0.25)
        self.declare_parameter('imu_gyro_noise_std', 0.002)
        self.declare_parameter('mag_noise_std', 1e-6)

        # 默认不发布 wheel_odom，避免和 Gazebo 插件自身发布的里程计冲突。
        self.declare_parameter('publish_wheel_odom', False)

        self.model_name = str(self.get_parameter('model_name').value)
        self.origin_lat = math.radians(float(self.get_parameter('origin_latitude').value))
        self.origin_lon = math.radians(float(self.get_parameter('origin_longitude').value))
        self.gps_noise_std = float(self.get_parameter('gps_noise_std').value)
        self.imu_gyro_noise_std = float(self.get_parameter('imu_gyro_noise_std').value)
        self.mag_noise_std = float(self.get_parameter('mag_noise_std').value)
        self.publish_wheel_odom = bool(self.get_parameter('publish_wheel_odom').value)

        self.gps_pub = self.create_publisher(NavSatFix, '/sensors/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, '/sensors/magnetic_field', 20)
        self.odom_pub = self.create_publisher(Odometry, '/sensors/wheel_odom', 20)

        self.last_speed = 0.0
        self.last_stamp = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_model_states, 10)
        self.get_logger().info('Synthetic GPS/IMU/magnetometer bridge enabled.')

    def local_xy_to_gps(self, x, y):
        """把 Gazebo world 米制坐标转换成近似 GPS 经纬度。"""
        noisy_x = x + random.gauss(0.0, self.gps_noise_std)
        noisy_y = y + random.gauss(0.0, self.gps_noise_std)
        lat = self.origin_lat + noisy_y / EARTH_RADIUS_M
        lon = self.origin_lon + noisy_x / (EARTH_RADIUS_M * math.cos(self.origin_lat))
        return math.degrees(lat), math.degrees(lon)

    def on_model_states(self, msg):
        """读取 Gazebo Classic 的真实模型状态，并发布合成传感器。"""
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[index]
        twist = msg.twist[index]
        yaw = quaternion_to_yaw(pose.orientation)

        # Gazebo model_states 中的 twist 是 world 坐标速度；
        # 定位融合需要车体前向速度，因此转换到 base_link。
        forward_speed, lateral_speed = world_to_body(twist.linear.x, twist.linear.y, yaw)

        stamp = self.get_clock().now()
        stamp_msg = stamp.to_msg()
        lat, lon = self.local_xy_to_gps(pose.position.x, pose.position.y)

        # 合成 GPS。
        gps = NavSatFix()
        gps.header.stamp = stamp_msg
        gps.header.frame_id = 'gps_link'
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = pose.position.z
        gps.position_covariance[0] = self.gps_noise_std ** 2
        gps.position_covariance[4] = self.gps_noise_std ** 2
        gps.position_covariance[8] = 0.5 ** 2
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(gps)

        # 根据前向速度差分估计纵向加速度。
        dt = 0.02
        if self.last_stamp is not None:
            dt = max(1e-3, (stamp - self.last_stamp).nanoseconds * 1e-9)
        accel_x = (forward_speed - self.last_speed) / dt
        self.last_speed = forward_speed
        self.last_stamp = stamp

        # 合成 IMU。orientation_covariance[0] = -1 表示 orientation 不作为可信姿态使用。
        imu = Imu()
        imu.header.stamp = stamp_msg
        imu.header.frame_id = 'imu_link'
        imu.orientation = yaw_to_quaternion(yaw)
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = twist.angular.z + random.gauss(0.0, self.imu_gyro_noise_std)
        imu.linear_acceleration.x = accel_x
        imu.linear_acceleration.y = lateral_speed * twist.angular.z
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        # 合成磁力计。这里用 yaw 构造一个水平磁场方向，仅用于兜底测试。
        field_strength = 2.5e-5
        mag = MagneticField()
        mag.header.stamp = stamp_msg
        mag.header.frame_id = 'magnetometer_link'
        mag.magnetic_field.x = field_strength * math.sin(yaw) + random.gauss(0.0, self.mag_noise_std)
        mag.magnetic_field.y = field_strength * math.cos(yaw) + random.gauss(0.0, self.mag_noise_std)
        mag.magnetic_field.z = -4.0e-5
        self.mag_pub.publish(mag)

        if self.publish_wheel_odom:
            # 仅在明确开启时发布，避免和真实 wheel odom 产生双发布冲突。
            odom = Odometry()
            odom.header.stamp = stamp_msg
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose = pose
            odom.twist.twist.linear.x = forward_speed
            odom.twist.twist.angular.z = twist.angular.z
            self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimSensorBridge()
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
