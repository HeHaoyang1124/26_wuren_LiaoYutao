import os
from glob import glob

from setuptools import find_packages, setup

"""right_angle_stack 的 Python 包安装配置。

ROS 2 的 ament_python 包需要在 setup.py 中声明：

- Python 模块；
- launch、urdf、rviz、config、SDF 模型等运行时资源；
- `ros2 run` 可调用的 console_scripts。

如果新增 launch、配置或模型文件，通常也要检查这里是否把它安装到了 share 目录。
"""


package_name = 'right_angle_stack'


def package_files(subdir, pattern):
    """返回某个子目录下匹配 pattern 的文件列表，用于 data_files 安装。"""
    return glob(os.path.join(subdir, pattern))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/urdf/rviz/config 需要安装到 share，launch 运行时通过 get_package_share_directory 查找。
        (os.path.join('share', package_name, 'launch'), package_files('launch', '*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), package_files('urdf', '*.xacro')),
        (os.path.join('share', package_name, 'rviz'), package_files('rviz', '*.rviz')),
        (os.path.join('share', package_name, 'config'), package_files('config', '*.yaml')),
        # Gazebo Sim 车辆 SDF 模型也要安装，否则 ros_gz_sim create 找不到。
        (
            os.path.join('share', package_name, 'models', 'right_angle_car_harmonic'),
            package_files(os.path.join('models', 'right_angle_car_harmonic'), 'model.*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'right_angle_car_wsl_headless'),
            package_files(os.path.join('models', 'right_angle_car_wsl_headless'), 'model.*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCUT Racing',
    maintainer_email='user@example.com',
    description='Right-angle Gazebo simulation stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 每个条目对应一个 ROS 2 Python 节点，可用 ros2 run 或 launch 启动。
            'cone_mapper = right_angle_stack.cone_mapper:main',
            'localization_fusion = right_angle_stack.localization_fusion:main',
            'pure_pursuit_controller = right_angle_stack.pure_pursuit_controller:main',
            'right_angle_planner = right_angle_stack.right_angle_planner:main',
            'sim_sensor_bridge = right_angle_stack.sim_sensor_bridge:main',
            'track_perception = right_angle_stack.track_perception:main',
        ],
    },
)
