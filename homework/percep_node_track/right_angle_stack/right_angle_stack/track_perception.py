import random

import rclpy
from fsd_common_msgs.msg import Cone, ConeDetections, Map
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .track_model import load_cones_from_sdf
from .utils import quaternion_to_yaw, world_to_body

"""内置 fallback 感知节点。

本节点保留为 fallback：当加密运行时、依赖或 ABI 出问题时，可以临时启用以验证
建图-规划-控制闭环。

功能：

- 读取赛道 SDF 中的所有锥桶；
- 根据 /localization/pose 得到车辆位置；
- 把 world 下锥桶转换到 base_link；
- 只发布车辆前方一定范围内的锥桶；
- 给位置加入高斯噪声，模拟感知误差；
"""


DEFAULT_CONES = [
    # 如果 SDF 解析失败，用这份手写锥桶列表兜底。
    # 坐标均为 world 坐标，颜色字符串和 fsd_common_msgs/Cone.color 一致。
    ('blue', -2.0, -15.0, 0.0), ('yellow', 2.0, -15.0, 0.0),
    ('blue', -2.0, -10.0, 0.0), ('yellow', 2.0, -10.0, 0.0),
    ('blue', -2.0, -5.0, 0.0), ('yellow', 2.0, -5.0, 0.0),
    ('blue', -2.0, 0.0, 0.0), ('yellow', 2.0, 0.0, 0.0),
    ('blue', -1.31, 4.32, 0.0), ('yellow', 2.76, 3.83, 0.0),
    ('blue', 0.68, 8.23, 0.0), ('yellow', 4.93, 7.07, 0.0),
    ('blue', 3.77, 11.32, 0.0), ('yellow', 8.17, 9.24, 0.0),
    ('blue', 7.68, 13.31, 0.0), ('yellow', 12.0, 10.0, 0.0),
    ('blue', 12.0, 14.0, 0.0), ('yellow', 17.0, 10.0, 0.0),
    ('blue', 17.0, 14.0, 0.0), ('yellow', 22.0, 10.0, 0.0),
    ('blue', 22.0, 14.0, 0.0), ('yellow', 27.0, 10.0, 0.0),
    ('blue', 27.0, 14.0, 0.0),
]


class TrackPerception(Node):
    def __init__(self):
        super().__init__('track_perception')

        # 赛道 SDF 路径。launch 中会传入 tracks/models/shixi/model.sdf。
        self.declare_parameter('track_sdf', '')

        # 感知范围：local_x 是车前方距离，local_y 是车左右距离。
        # rear_margin 允许略微看到车身后方一点点，避免锥桶刚经过车身时突然消失。
        self.declare_parameter('max_range', 18.0)
        self.declare_parameter('lateral_range', 9.0)
        self.declare_parameter('rear_margin', 1.0)

        # 位置噪声，用来模拟感知子系统的测量误差。
        self.declare_parameter('position_noise_std', 0.05)
        self.declare_parameter('publish_rate', 10.0)

        # 输出话题做成参数，便于和老师给的 sim_perception 或其他感知包对齐。
        self.declare_parameter('map_topic', '/perception/cones')
        self.declare_parameter('detections_topic', '/perception/cone_detections')

        self.max_range = float(self.get_parameter('max_range').value)
        self.lateral_range = float(self.get_parameter('lateral_range').value)
        self.rear_margin = float(self.get_parameter('rear_margin').value)
        self.noise_std = float(self.get_parameter('position_noise_std').value)

        track_sdf = str(self.get_parameter('track_sdf').value)
        if track_sdf:
            try:
                self.cones = load_cones_from_sdf(track_sdf)
            except Exception as exc:
                self.get_logger().warn(f'Failed to load track SDF, using built-in cone list: {exc}')
                self.cones = DEFAULT_CONES
        else:
            self.cones = DEFAULT_CONES

        self.x = 0.0
        self.y = -15.0
        self.yaw = 1.57079632679

        self.map_pub = self.create_publisher(Map, self.get_parameter('map_topic').value, 10)
        self.det_pub = self.create_publisher(ConeDetections, self.get_parameter('detections_topic').value, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose, 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self.on_timer)
        self.get_logger().info(f'Track perception publishing {len(self.cones)} known cones in base_link frame.')

    def on_pose(self, msg):
        """接收定位输出，更新车辆在 world 下的位姿。"""
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.orientation)

    def make_local_cone(self, color, local_x, local_y, z):
        """构造一个 base_link 下的锥桶观测。

        local_x/local_y 已经是车辆坐标系下的位置。
        加入高斯噪声，并设置置信度字段。
        """
        cone = Cone()
        cone.position.x = local_x + random.gauss(0.0, self.noise_std)
        cone.position.y = local_y + random.gauss(0.0, self.noise_std)
        cone.position.z = z
        cone.color = color
        cone.pose_confidence = 0.9
        cone.color_confidence = 0.98
        return cone

    def on_timer(self):
        """周期性发布局部锥桶。

        每一帧都遍历静态赛道锥桶，根据车辆当前位姿把锥桶转到 base_link，然后筛选前方范围内的锥桶，模拟感知。
        """
        stamp = self.get_clock().now().to_msg()
        cone_map = Map()
        cone_map.header.stamp = stamp

        # 感知输出是局部观测，因此 frame_id 必须是 base_link。
        # 后续 cone_mapper 会根据 /localization/pose 转回 world。
        cone_map.header.frame_id = 'base_link'

        detections = ConeDetections()
        detections.header = cone_map.header

        for color, wx, wy, wz in self.cones:
            # world -> base_link：判断锥桶相对车辆的前后左右。
            local_x, local_y = world_to_body(wx - self.x, wy - self.y, self.yaw)

            # 过滤车后方或过远锥桶。
            if local_x < -self.rear_margin or local_x > self.max_range:
                continue
            # 过滤横向距离过大的锥桶，模拟传感器视场范围。
            if abs(local_y) > self.lateral_range:
                continue

            cone = self.make_local_cone(color, local_x, local_y, wz)
            detections.cone_detections.append(cone)

            # Map 消息按颜色分组；ConeDetections 则是单一数组。
            if color == 'blue':
                cone_map.cone_blue.append(cone)
            elif color == 'yellow':
                cone_map.cone_yellow.append(cone)
            elif color == 'red':
                cone_map.cone_red.append(cone)
            else:
                cone_map.cone_unknown.append(cone)

        self.map_pub.publish(cone_map)
        self.det_pub.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPerception()
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
