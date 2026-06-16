import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from .utils import normalize_angle, quaternion_to_yaw, world_to_body

"""Pure Pursuit 控制器。

输入：

- /planning/centerline：规划器输出路径。
- /localization/odom：车辆当前位姿和速度。

输出：

- /cmd_vel：Gazebo DiffDrive 接收的速度控制指令。

"""


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # 目标速度和最小速度，用来控制直道/弯道的行驶节奏。
        self.declare_parameter('target_speed', 3.0)
        self.declare_parameter('min_speed', 1.2)

        # 目标点必须在车前方且距离当前车体有一定前瞻距离。
        self.declare_parameter('lookahead_distance', 4.0)
        self.declare_parameter('max_yaw_rate', 1.4)
        self.declare_parameter('stop_distance', 1.2)

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.path = []
        self.state = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/planning/centerline', self.on_path, 10)
        self.create_subscription(Odometry, '/localization/odom', self.on_odom, 20)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info('Pure pursuit controller started.')

    def on_path(self, msg):
        """把 Path 消息简化成点列表，便于后续快速计算。"""
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    def on_odom(self, msg):
        """缓存当前车辆位姿。"""
        self.state = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def publish_stop(self):
        """发送零速度指令。"""
        self.cmd_pub.publish(Twist())

    def on_timer(self):
        """周期性计算控制命令。

        流程：

        1. 找最近路径点；
        2. 找前方 lookahead 距离外的目标点；
        3. 计算局部坐标下的曲率；
        4. 根据曲率生成角速度；
        5. 根据曲率适当降速。
        """
        if self.state is None or len(self.path) < 2:
            self.publish_stop()
            return

        x, y, yaw = self.state

        # 当前车辆到路径上每个点的欧式距离，取最近点作为参考。
        nearest_index = min(
            range(len(self.path)),
            key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y),
        )

        goal_x, goal_y = self.path[-1]

        # 已接近终点时停车。
        if nearest_index >= len(self.path) - 4 and math.hypot(goal_x - x, goal_y - y) < self.stop_distance:
            self.publish_stop()
            return

        # 从最近点往后找一个在车前方、且距离足够远的 lookahead 目标点。
        target = self.path[-1]
        for point in self.path[nearest_index:]:
            dx = point[0] - x
            dy = point[1] - y
            local_x, _ = world_to_body(dx, dy, yaw)
            if local_x > 0.2 and math.hypot(dx, dy) >= self.lookahead_distance:
                target = point
                break

        local_x, local_y = world_to_body(target[0] - x, target[1] - y, yaw)
        lookahead = max(0.5, math.hypot(local_x, local_y))

        # Pure Pursuit 核心公式：根据局部目标点求曲率。
        curvature = 2.0 * local_y / (lookahead * lookahead)

        # yaw_error 作为辅助修正项，帮助在中等曲率下更快对齐。
        yaw_error = math.atan2(local_y, max(local_x, 1e-3))

        yaw_rate = self.target_speed * curvature
        yaw_rate += 0.20 * normalize_angle(yaw_error)
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))

        # 弯道时自动降速。
        turn_slowdown = min(1.0, abs(curvature) / 0.28)
        speed = self.target_speed * (1.0 - 0.45 * turn_slowdown)
        speed = max(self.min_speed, speed)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
