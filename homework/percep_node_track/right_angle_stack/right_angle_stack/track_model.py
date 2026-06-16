import xml.etree.ElementTree as ET

"""读取赛道 SDF 中的锥桶布置。

赛道模型 tracks/models/shixi/model.sdf 通过多个 <include> 引用蓝锥和黄锥模型。
内置 fallback 感知节点需要知道这些锥桶在 world 坐标系下的位置，因此这里直接解析 SDF。

注意：

- 这里不解析 Gazebo 运行时状态，只解析静态赛道文件。
- 返回坐标保持 SDF 中的 world 坐标。
- 颜色根据 include 的 uri 和 name 推断。
"""


def load_cones_from_sdf(path):
    """从 SDF 文件中提取锥桶列表。

    返回格式：
        ('blue', x, y, z),
        ('yellow', x, y, z),
        ...

    track_perception 基于静态锥桶和车辆当前位姿，模拟车辆前方范围内的感知。
    """
    tree = ET.parse(path)
    root = tree.getroot()
    cones = []
    for include in root.findall('.//include'):
        # uri 指向模型类型；
        # name 是实例名。
        uri = include.findtext('uri', default='')
        name = include.findtext('name', default='')
        pose_text = include.findtext('pose', default='0 0 0 0 0 0')
        pose_values = [float(value) for value in pose_text.split()]

        # SDF pose 的前三项是 x/y/z，后三项是 roll/pitch/yaw。
        x = pose_values[0] if len(pose_values) > 0 else 0.0
        y = pose_values[1] if len(pose_values) > 1 else 0.0
        z = pose_values[2] if len(pose_values) > 2 else 0.0

        label = f'{uri} {name}'.lower()
        if 'blue' in label:
            color = 'blue'
        elif 'yellow' in label:
            color = 'yellow'
        elif 'red' in label:
            color = 'red'
        else:
            color = 'unknown'

        cones.append((color, x, y, z))
    return cones
