import math

import rclpy
from fsd_common_msgs.msg import Cone, ConeDetections, Map
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .utils import body_to_world, color_key, quaternion_to_yaw, set_marker_color

"""锥桶建图节点。

这个节点负责把“车辆局部感知到的锥桶”累积成 `world` 坐标系下的全局地图。

输入：

- `/localization/pose`：车辆当前在 world 中的位姿。
- `/perception/cones`：局部锥桶地图，frame 通常是 `base_link`。
- `/perception/cone_detections`：单帧局部锥桶列表。

输出：

- `/estimation/slam/map`：全局锥桶地图，供规划使用。
- `/visualization/cone_map`：RViz 可视化 marker。

它不是严格意义上的 SLAM，而是一个针对赛道任务的轻量地标融合器：

- 把局部观测转到 world；
- 按颜色分桶；
- 对近距离重复观测做简单融合。
"""


class ConeMapper(Node):
    def __init__(self):
        super().__init__('cone_mapper')

        # 感知消息话题可配置，便于和内置感知或老师给的感知包对齐。
        self.declare_parameter('perception_map_topic', '/perception/cones')
        self.declare_parameter('perception_detections_topic', '/perception/cone_detections')

        # merge_distance 越大，越容易把两次观测合并为同一个地标。
        self.declare_parameter('merge_distance', 0.75)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')

        self.merge_distance = float(self.get_parameter('merge_distance').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pose = None
        self.landmarks = {
            'blue': [],
            'yellow': [],
            'red': [],
            'unknown': [],
        }

        # 全局地图输出给规划器，marker 输出给 RViz。
        self.map_pub = self.create_publisher(Map, '/estimation/slam/map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/cone_map', 10)

        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose, 10)
        self.create_subscription(
            Map,
            self.get_parameter('perception_map_topic').value,
            self.on_map_detection,
            10,
        )
        self.create_subscription(
            ConeDetections,
            self.get_parameter('perception_detections_topic').value,
            self.on_cone_detections,
            10,
        )
        self.create_timer(0.2, self.publish_outputs)
        self.get_logger().info('Cone mapper started. Local perception is transformed into the world ENU frame.')

    def on_pose(self, msg):
        """记录当前车辆在 world 下的位姿，用于把局部锥桶投到世界坐标系。"""
        self.pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            quaternion_to_yaw(msg.pose.orientation),
        )

    def on_map_detection(self, msg):
        """处理按颜色分组的锥桶地图消息。"""
        self.process_cones(msg.header.frame_id, msg.cone_blue, 'blue')
        self.process_cones(msg.header.frame_id, msg.cone_yellow, 'yellow')
        self.process_cones(msg.header.frame_id, msg.cone_red, 'red')
        self.process_cones(msg.header.frame_id, msg.cone_unknown, 'unknown')
        self.publish_outputs()

    def on_cone_detections(self, msg):
        """处理不分颜色的单帧锥桶检测消息。"""
        self.process_cones(msg.header.frame_id, msg.cone_detections, None)
        self.publish_outputs()

    def process_cones(self, frame_id, cones, fallback_color):
        """遍历一组锥桶并投到 world 坐标系。"""
        for cone in cones:
            key = color_key(cone.color or fallback_color)
            transformed = self.to_world(frame_id, cone)
            if transformed is None:
                continue
            wx, wy = transformed
            self.merge_landmark(key, wx, wy, cone.pose_confidence, cone.color_confidence)

    def to_world(self, frame_id, cone):
        """把单个锥桶从局部坐标转换到 world 坐标。

        如果输入消息本身就是 world/map 坐标，则直接返回。
        如果是 base_link 坐标，则需要借助当前车辆位姿进行旋转和平移。
        """
        frame = (frame_id or self.base_frame).strip('/')
        if frame in (self.world_frame, 'map'):
            return cone.position.x, cone.position.y
        if self.pose is None:
            return None
        x, y, yaw = self.pose
        return body_to_world(cone.position.x, cone.position.y, x, y, yaw)

    def merge_landmark(self, color, x, y, pose_confidence, color_confidence):
        """把新观测和已有地标做简单融合。

        这里不是复杂滤波，只做最近邻匹配 + 加权平均，足够满足赛道任务。
        """
        bucket = self.landmarks[color]
        closest = None
        best_distance = float('inf')
        for landmark in bucket:
            dist = math.hypot(landmark['x'] - x, landmark['y'] - y)
            if dist < best_distance:
                best_distance = dist
                closest = landmark

        if closest is None or best_distance > self.merge_distance:
            bucket.append({
                'x': x,
                'y': y,
                'count': 1,
                'pose_confidence': pose_confidence,
                'color_confidence': color_confidence,
            })
            return

        count = min(closest['count'] + 1, 30)
        alpha = 1.0 / count
        closest['x'] = (1.0 - alpha) * closest['x'] + alpha * x
        closest['y'] = (1.0 - alpha) * closest['y'] + alpha * y
        closest['count'] = count
        closest['pose_confidence'] = max(closest['pose_confidence'], pose_confidence)
        closest['color_confidence'] = max(closest['color_confidence'], color_confidence)

    def make_cone(self, color, landmark):
        """把内部地标字典转换成对外发布的 Cone 消息。"""
        cone = Cone()
        cone.position.x = landmark['x']
        cone.position.y = landmark['y']
        cone.position.z = 0.0
        cone.color = color
        cone.pose_confidence = float(landmark['pose_confidence'])
        cone.color_confidence = float(landmark['color_confidence'])
        return cone

    def publish_outputs(self):
        """周期性发布全局地图和 RViz marker。"""
        stamp = self.get_clock().now().to_msg()
        msg = Map()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.cone_blue = [self.make_cone('blue', item) for item in self.landmarks['blue']]
        msg.cone_yellow = [self.make_cone('yellow', item) for item in self.landmarks['yellow']]
        msg.cone_red = [self.make_cone('red', item) for item in self.landmarks['red']]
        msg.cone_unknown = [self.make_cone('unknown', item) for item in self.landmarks['unknown']]
        self.map_pub.publish(msg)

        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        for color, bucket in self.landmarks.items():
            for landmark in bucket:
                marker = Marker()
                marker.header = msg.header
                marker.ns = f'{color}_cones'
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                # marker 的位置和大小只是可视化表达，不影响地图数据本身。
                marker.pose.position.x = landmark['x']
                marker.pose.position.y = landmark['y']
                marker.pose.position.z = 0.28
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.35
                marker.scale.y = 0.35
                marker.scale.z = 0.56
                set_marker_color(marker, color)
                markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapper()
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
