import math

import rclpy
from fsd_common_msgs.msg import Map
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .utils import yaw_to_quaternion

"""右角弯中心线规划节点。

输入：

- /estimation/slam/map：全局锥桶地图。
- /localization/pose：车辆当前位姿。

输出：

- /planning/centerline：给 Pure Pursuit 使用的路径。
- /visualization/planning：RViz 路径 marker。

给规划器设计了两套路径生成方案：

1. 优先从蓝锥、黄锥配对求中点，得到锥桶中心线；
2. 如果地图不足，则回退到解析路径，保证车辆仍能跑完整个直角弯。
"""


class RightAnglePlanner(Node):
    def __init__(self):
        super().__init__('right_angle_planner')

        # prefer_cone_map=True 时优先使用地图中心线，而不是解析 fallback。
        self.declare_parameter('prefer_cone_map', True)
        self.declare_parameter('fallback_path', True)

        # 这三个参数定义 fallback 路径中的右角弯几何。
        self.declare_parameter('turn_center_x', 12.0)
        self.declare_parameter('turn_center_y', 0.0)
        self.declare_parameter('turn_radius', 12.0)

        # 配对距离和路径加密步长。
        self.declare_parameter('pair_distance_max', 6.2)
        self.declare_parameter('path_step', 0.8)

        self.prefer_cone_map = bool(self.get_parameter('prefer_cone_map').value)
        self.use_fallback = bool(self.get_parameter('fallback_path').value)
        self.center_x = float(self.get_parameter('turn_center_x').value)
        self.center_y = float(self.get_parameter('turn_center_y').value)
        self.radius = float(self.get_parameter('turn_radius').value)
        self.pair_distance_max = float(self.get_parameter('pair_distance_max').value)
        self.path_step = float(self.get_parameter('path_step').value)

        self.latest_map = None
        self.current_pose = None
        self.path_pub = self.create_publisher(Path, '/planning/centerline', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/planning', 10)
        self.create_subscription(Map, '/estimation/slam/map', self.on_map, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose, 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info('Right-angle centerline planner started.')

    def on_map(self, msg):
        """缓存最新地图。"""
        self.latest_map = msg

    def on_pose(self, msg):
        """缓存当前位姿，当前主要用于未来扩展，不直接参与路径计算。"""
        self.current_pose = msg

    def track_progress(self, point):
        """给路径点沿赛道前进顺序做标记。"""
        x, y = point
        if y <= 0.2 and x < 3.0:
            return y + 15.0
        theta = math.atan2(y - self.center_y, x - self.center_x)
        if x <= self.center_x + 1.0 and y >= -0.5:
            theta = min(math.pi, max(math.pi / 2.0, theta))
            return 15.0 + (math.pi - theta) * self.radius
        return 15.0 + 0.5 * math.pi * self.radius + max(0.0, x - self.center_x)

    def analytic_path(self):
        """生成直角弯的解析 fallback 路径。

        - 直道：x=0，y 从 -15 到 0；
        - 弯道：以 (turn_center_x, turn_center_y) 为圆心的四分之一圆；
        - 出弯直道：y = turn_center_y + radius，x 向前延伸。
        """
        points = []
        y = -15.0
        while y <= 0.0:
            points.append((0.0, y))
            y += self.path_step

        theta = math.pi
        theta_end = math.pi / 2.0
        dtheta = self.path_step / max(self.radius, 0.1)
        while theta >= theta_end:
            x = self.center_x + self.radius * math.cos(theta)
            y = self.center_y + self.radius * math.sin(theta)
            points.append((x, y))
            theta -= dtheta

        x = self.center_x
        while x <= 32.0:
            points.append((x, self.center_y + self.radius))
            x += self.path_step
        return points

    def cone_centerline(self):
        """根据蓝锥、黄锥配对得到中心线。
        思路：

        - 一个蓝锥只配一个最近的黄锥；
        - 配对距离太大就跳过；
        - 取中点作为中心线点；
        - 中点数量足够后生成路径。
        """
        if self.latest_map is None:
            return []
        blue = [(c.position.x, c.position.y) for c in self.latest_map.cone_blue]
        yellow = [(c.position.x, c.position.y) for c in self.latest_map.cone_yellow]
        if len(blue) < 3 or len(yellow) < 3:
            return []

        used_yellow = set()
        midpoints = []
        for bx, by in blue:
            best_index = None
            best_dist = float('inf')
            for index, (yx, yy) in enumerate(yellow):
                if index in used_yellow:
                    continue
                dist = math.hypot(bx - yx, by - yy)
                if dist < best_dist:
                    best_dist = dist
                    best_index = index
            if best_index is None or best_dist > self.pair_distance_max:
                continue
            used_yellow.add(best_index)
            yx, yy = yellow[best_index]
            midpoints.append(((bx + yx) * 0.5, (by + yy) * 0.5))

        midpoints = sorted(midpoints, key=self.track_progress)
        if len(midpoints) < 5:
            return []
        return self.densify(midpoints)

    def densify(self, points):
        """对稀疏路径点做线性插值，让 Pure Pursuit 更稳定。"""
        if not points:
            return []
        dense = [points[0]]
        for start, end in zip(points, points[1:]):
            sx, sy = start
            ex, ey = end
            dist = math.hypot(ex - sx, ey - sy)
            steps = max(1, int(dist / self.path_step))
            for i in range(1, steps + 1):
                ratio = i / steps
                dense.append((sx + ratio * (ex - sx), sy + ratio * (ey - sy)))
        return dense

    def choose_path(self):
        """选择本周期最终输出路径。

        1. 锥桶地图中心线；
        2. 备用的解析路径；
        3. 没有路径则不发布。
        """
        cone_path = self.cone_centerline() if self.prefer_cone_map else []
        if len(cone_path) >= 8:
            return cone_path, 'cone_map'
        if self.use_fallback:
            return self.analytic_path(), 'analytic'
        return [], 'none'

    def on_timer(self):
        """周期性生成 Path 并发布 marker。"""
        points, source = self.choose_path()
        if not points:
            return
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'world'
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(points):
                nx, ny = points[index + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif index > 0:
                px, py = points[index - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.publish_markers(path.header, points, source)

    def publish_markers(self, header, points, source):
        """中心线路径在 RViz 中可视化。"""
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header = header
        line.ns = 'centerline'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.12
        line.color.a = 1.0
        if source == 'cone_map':
            line.color.r = 0.0
            line.color.g = 0.9
            line.color.b = 0.35
        else:
            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 1.0
        for x, y in points:
            line.points.append(Point(x=x, y=y, z=0.05))
        markers.markers.append(line)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RightAnglePlanner()
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
