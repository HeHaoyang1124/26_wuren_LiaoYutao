import math

from geometry_msgs.msg import Quaternion

"""right_angle_stack 的通用数学工具。

本文件只放不依赖 ROS 节点状态的纯函数，主要服务于：

- 航向角与四元数互相转换；
- world 全局坐标系与 base_link 车辆局部坐标系互相转换；
- 锥桶颜色字符串归一化；
- RViz marker 颜色设置。

坐标系约定：

- world：ENU，x 向东，y 向北，z 向上。
- base_link：FLU，x 向前，y 向左，z 向上。

这些函数会被定位、感知、建图、规划、控制多个节点复用，所以保持成无副作用函数，方便单独检查和答辩解释。
"""


def normalize_angle(angle):
    """把任意角度归一化到 [-pi, pi)。

    角度积分、航向误差、磁力计修正都可能产生超过 pi 的角度。
    控制器只关心最短旋转方向，因此需要把误差压回标准范围。
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    """把平面 yaw 角转换成 ROS 四元数。

    本车只在平面上运动，roll 和 pitch 默认都是 0，因此四元数里只有 z 和 w 分量非零。
    该函数用于 /localization/pose、/localization/odom、TF 和规划路径姿态。
    """
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    """从 ROS 四元数中提取平面 yaw 角。

    Gazebo、Odometry、PoseStamped 中姿态都以四元数表示。
    建图和控制只需要平面航向，因此统一在这里转换，避免各节点重复写公式。
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(dx, dy, yaw):
    """把 world 坐标差转换到车辆 base_link 坐标系。

    参数 dx, dy 是目标点相对车辆当前位置的 world 坐标差。
    返回值：

    - local_x：目标在车辆前后方向的位置，正值表示在车前方。
    - local_y：目标在车辆左右方向的位置，正值表示在车左侧。

    感知节点用它判断锥桶是否位于车辆前方范围内；
    控制器用它判断路径目标点是否在车前方以及需要向左/右转。
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def body_to_world(local_x, local_y, origin_x, origin_y, yaw):
    """把车辆局部坐标点转换到 world 坐标系。

    建图节点收到的锥桶通常在 base_link 下，需要结合车辆在 world 下的
    位置和 yaw，把局部观测变成全局地标。
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        origin_x + c * local_x - s * local_y,
        origin_y + s * local_x + c * local_y,
    )


def color_key(color):
    """把颜色字符串归一化成固定分类。

    感知包可能发布 blue、BLUE、yellow_cone 等不同字符串。
    建图和规划只需要稳定的颜色桶，因此统一映射到：
    blue、yellow、red、unknown。
    """
    value = (color or '').lower()
    if 'blue' in value:
        return 'blue'
    if 'yellow' in value:
        return 'yellow'
    if 'red' in value:
        return 'red'
    return 'unknown'


def set_marker_color(marker, color):
    """根据锥桶颜色设置 RViz marker 的 RGBA。

    这里只负责可视化颜色，不影响 Gazebo 模型材质，也不影响建图数据。
    """
    key = color_key(color)
    marker.color.a = 0.95
    if key == 'blue':
        marker.color.r = 0.02
        marker.color.g = 0.16
        marker.color.b = 1.0
    elif key == 'yellow':
        marker.color.r = 1.0
        marker.color.g = 0.78
        marker.color.b = 0.02
    elif key == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.04
        marker.color.b = 0.02
    else:
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
