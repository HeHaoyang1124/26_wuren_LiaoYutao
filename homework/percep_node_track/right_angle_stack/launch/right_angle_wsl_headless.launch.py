import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

"""WSL/headless 调试入口。

该入口使用不含相机和 GPU 雷达的轻量车辆模型，适合在 WSL 图形栈不稳定时验证：

- Gazebo server 是否能启动；
- 车辆是否能 spawn；
- `/cmd_vel` bridge 是否生效；
- 定位、感知、建图、规划、控制链路是否闭环。

默认感知仍然使用老师给的 `sim_perception`。
"""


def generate_launch_description():
    # 所有资源路径都从安装后的 package share 中获取，避免依赖源码路径。
    stack_share = get_package_share_directory('right_angle_stack')
    track_share = get_package_share_directory('right_angle_track')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_name = 'right_angle_world'
    world_path = os.path.join(track_share, 'worlds', 'right_angle_wsl_headless.sdf')
    # headless 模型去掉了渲染传感器，降低 WSL 下 OGRE/GUI 崩溃概率。
    robot_sdf = os.path.join(stack_share, 'models', 'right_angle_car_wsl_headless', 'model.sdf')
    robot_xacro = os.path.join(stack_share, 'urdf', 'right_angle_car.urdf.xacro')
    rviz_config = os.path.join(stack_share, 'rviz', 'right_angle.rviz')
    stack_config = os.path.join(stack_share, 'config', 'right_angle_stack.yaml')
    track_sdf = os.path.join(track_share, 'models', 'shixi', 'model.sdf')
    track_models = os.path.join(track_share, 'models')
    stack_models = os.path.join(stack_share, 'models')

    model_name = LaunchConfiguration('model_name')
    gz_args = LaunchConfiguration('gz_args')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_builtin_perception = LaunchConfiguration('use_builtin_perception')
    use_sim_perception = LaunchConfiguration('use_sim_perception')
    perception_map_topic = LaunchConfiguration('perception_map_topic')
    perception_detections_topic = LaunchConfiguration('perception_detections_topic')
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    gz_resource_path = [
        track_models,
        os.pathsep,
        stack_models,
        os.pathsep,
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='racecar'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r -s -v 4 ', world_path],
            description='Gazebo Sim server-only arguments for WSL headless debugging.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # 默认感知使用老师给的 sim_perception；内置感知只作 fallback。
        DeclareLaunchArgument('use_builtin_perception', default_value='false'),
        DeclareLaunchArgument('use_sim_perception', default_value='true'),
        DeclareLaunchArgument('perception_map_topic', default_value='/perception/cones'),
        DeclareLaunchArgument('perception_detections_topic', default_value='/perception/cone_detections'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gz_resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': gz_args,
            }.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_right_angle_car',
            output='screen',
            arguments=[
                # 起点：(0, -15)，朝北 yaw=pi/2。
                '-world', world_name,
                '-file', robot_sdf,
                '-name', model_name,
                '-x', '0.0',
                '-y', '-15.0',
                '-z', '0.0',
                '-Y', '1.57079632679',
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_clock_bridge',
            output='screen',
            arguments=[
                # 把 Gazebo world clock 映射到 ROS /clock。
                f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                # headless 入口只桥接非渲染传感器，避免相机/GPU 雷达触发渲染线程。
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/sensors/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/sensors/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/sensors/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/sensors/magnetic_field@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', robot_xacro]),
                'use_sim_time': sim_time_param,
            }],
        ),
        Node(
            package='right_angle_stack',
            executable='localization_fusion',
            name='localization_fusion',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),
        Node(
            package='right_angle_stack',
            executable='track_perception',
            name='track_perception',
            output='screen',
            parameters=[stack_config, {
                'track_sdf': track_sdf,
                'map_topic': perception_map_topic,
                'detections_topic': perception_detections_topic,
                'use_sim_time': sim_time_param,
            }],
            condition=IfCondition(use_builtin_perception),
        ),
        Node(
            package='sim_perception',
            executable='sim_node',
            name='sim_perception',
            output='screen',
            # 默认感知节点也使用仿真时钟。
            parameters=[{'use_sim_time': sim_time_param}],
            condition=IfCondition(use_sim_perception),
        ),
        Node(
            package='right_angle_stack',
            executable='cone_mapper',
            name='cone_mapper',
            output='screen',
            parameters=[stack_config, {
                'perception_map_topic': perception_map_topic,
                'perception_detections_topic': perception_detections_topic,
                'use_sim_time': sim_time_param,
            }],
        ),
        Node(
            package='right_angle_stack',
            executable='right_angle_planner',
            name='right_angle_planner',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),
        Node(
            package='right_angle_stack',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[stack_config, {'use_sim_time': sim_time_param}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
