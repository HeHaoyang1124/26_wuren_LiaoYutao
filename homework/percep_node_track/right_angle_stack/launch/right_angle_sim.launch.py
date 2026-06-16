import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

"""Gazebo Classic 兼容入口。

当前在 Jazzy + Gazebo Sim 8上运行成功，本文件保留作为旧环境。
如果环境中没有 gazebo_ros，这个入口会失败；Jazzy 主调试用right_angle_harmonic.launch.py。
"""


def generate_launch_description():
    # Classic 路线使用 xacro + gazebo_ros spawn_entity。
    stack_share = get_package_share_directory('right_angle_stack')
    track_share = get_package_share_directory('right_angle_track')

    robot_xacro = os.path.join(stack_share, 'urdf', 'right_angle_car.urdf.xacro')
    rviz_config = os.path.join(stack_share, 'rviz', 'right_angle.rviz')
    stack_config = os.path.join(stack_share, 'config', 'right_angle_stack.yaml')
    track_sdf = os.path.join(track_share, 'models', 'shixi', 'model.sdf')

    model_name = LaunchConfiguration('model_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_builtin_perception = LaunchConfiguration('use_builtin_perception')
    use_sim_perception = LaunchConfiguration('use_sim_perception')
    use_synthetic_sensors = LaunchConfiguration('use_synthetic_sensors')
    perception_map_topic = LaunchConfiguration('perception_map_topic')
    perception_detections_topic = LaunchConfiguration('perception_detections_topic')
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    robot_description = {
        'robot_description': Command(['xacro', ' ', robot_xacro])
    }

    gazebo_models = PathJoinSubstitution([FindPackageShare('right_angle_track'), 'models'])

    def gazebo_actions(context):
        """根据 use_gazebo 参数决定是否启动 Gazebo Classic。
        避免在WSL上调试图形界面崩溃导致看不到其他节点运行是否正常。
        用 OpaqueFunction 是为了在 launch 执行期再检查 gazebo_ros，
        这样 use_gazebo:=false 时可以只跑算法看反馈。
        """
        use_gazebo_value = use_gazebo.perform(context).strip().lower()
        if use_gazebo_value not in ('1', 'true', 'yes', 'on'):
            return []

        get_package_share_directory('gazebo_ros')
        world_path = os.path.join(track_share, 'worlds', 'right_angle.world')
        gzserver = ExecuteProcess(
            cmd=[
                'gzserver',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path,
            ],
            output='screen',
        )
        gzclient = ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
        )
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', model_name,
                '-x', '0.0',
                '-y', '-15.0',
                '-z', '0.02',
                '-Y', '1.57079632679',
            ],
            output='screen',
        )
        return [gzserver, gzclient, spawn_node]

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='racecar'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_builtin_perception', default_value='false'),
        DeclareLaunchArgument('use_sim_perception', default_value='true'),
        DeclareLaunchArgument('use_synthetic_sensors', default_value='false'),
        DeclareLaunchArgument('perception_map_topic', default_value='/perception/cones'),
        DeclareLaunchArgument('perception_detections_topic', default_value='/perception/cone_detections'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[gazebo_models, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
        ),
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        OpaqueFunction(function=gazebo_actions),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': sim_time_param}],
        ),
        Node(
            package='right_angle_stack',
            executable='sim_sensor_bridge',
            name='sim_sensor_bridge',
            output='screen',
            parameters=[stack_config, {
                'model_name': model_name,
                'use_sim_time': sim_time_param,
            }],
            condition=IfCondition(use_synthetic_sensors),
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
