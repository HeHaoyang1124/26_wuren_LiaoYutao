import math

from geometry_msgs.msg import Quaternion

"""right_angle_stack 的通用数学工具。

- 航向角与四元数互相转换；
- world 全局坐标系与 base_link 车辆局部坐标系互相转换；
- 锥桶颜色字符串归一化；
- RViz marker 颜色设置。

坐标系约定：

- world：ENU，x 向东，y 向北，z 向上。
- base_link：FLU，x 向前，y 向左，z 向上。

"""


def normalize_angle(angle):
    """归一化角度到 [-pi, pi)。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    """把平面 yaw 角转换成 ROS 四元数。"""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    """从 ROS 四元数中提取平面 yaw 角。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(dx, dy, yaw):
    """把 world 坐标差转换到车辆 base_link 坐标系。

    参数 dx, dy 是目标点相对车辆当前位置的 world 坐标差。
    返回值：

    - local_x：目标在车辆前后方向的位置，正值表示在车前方。
    - local_y：目标在车辆左右方向的位置，正值表示在车左侧。
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
    """把颜色字符串转换成固定分类，应对感知可能传入的字符串。"""
    value = (color or '').lower()
    if 'blue' in value:
        return 'blue'
    if 'yellow' in value:
        return 'yellow'
    if 'red' in value:
        return 'red'
    return 'unknown'


def set_marker_color(marker, color):
    """根据锥桶颜色设置 RViz marker 的 RGBA。"""
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
