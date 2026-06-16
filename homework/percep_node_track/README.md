# Gazebo 仿真任务：直角弯

本工程完成一个 ROS 2 + Gazebo Sim 的直角弯仿真闭环：

```text
Gazebo 赛道/车辆/传感器
  -> 定位融合
  -> 锥桶感知输入
  -> 建图
  -> 中心线规划
  -> Pure Pursuit 控制
  -> /cmd_vel
  -> Gazebo 车辆运动
```

当前在以下环境成功通过测试：

- ROS 2 Jazzy
- Gazebo Sim 8
- ros_gz_sim + ros_gz_bridge
- 工作区：homework/percep_node_track
- 起点(0, -15)，车辆朝北，初始 yaw 为$\frac{\pi}{2}$

在调试过程中尝试在WSL和实体机上运行，humble与jazzy版本的ros2都有使用，最终在Jazzy + Gazebo Sim 8 上首先跑通。
总结调试环境使用经验如下：

1. 尽量选用实体机（双系统应该也可以而且我（廖宇韬）常用的工作机带显卡，仿真效果更好），WSL上的内存管理和显卡驱动配置问题很多，经常刚启动Gazebo后就闪退；
2. jazzy相比humble，对msg的审查更严格，最初的msg走CMake时生成不了目标文件，最后借助AI给出了目标idl产物，才得以在jazzy上通过；
3. Gazebo分Classic和版本更加新的Harmonic等，两版本间不兼容的部分略多，起初在WSL上用Classic,出现了兼容性问题，后来升级后在WSL和实体机上兼容性问题消失。
4. Gazebo疑似对路径名称很敏感，分析报错时有审查路径名的内容，于是尽可能把文件名和文件夹名替换成了更安全的版本（比如作业文件夹的加号换成了下划线）

## 坐标系约定

课程给出的感知代码对坐标系有要求，本工程按下面约定实现：

- 全局坐标系：world
- 全局坐标系方向：ENU，x 向东，y 向北，z 向上
- 车辆底盘坐标系：base_link
- 车辆局部坐标系方向：FLU，x 向前，y 向左，z 向上
- 相机光学坐标系：camera_optical_frame
- 雷达坐标系：lidar_link
- 定位 TF：world -> base_link

车辆从 (0, -15) 出发，朝北行驶。因为 world 的 y 轴向北，所以车辆初始 yaw 是 $\frac{\pi}{2}$。

## 工程结构

```text
percep_node_track/
├── fsd_common_msgs/
│   └── msg/
│       ├── Cone.idl
│       ├── ConeDetections.idl
│       └── Map.idl
├── right_angle_stack/
│   ├── config/right_angle_stack.yaml
│   ├── launch/
│   │   ├── right_angle_harmonic.launch.py
│   │   ├── right_angle_wsl_headless.launch.py
│   │   └── right_angle_sim.launch.py
│   ├── models/
│   │   ├── right_angle_car_harmonic/model.sdf
│   │   └── right_angle_car_wsl_headless/model.sdf
│   ├── right_angle_stack/
│   │   ├── localization_fusion.py
│   │   ├── track_perception.py
│   │   ├── cone_mapper.py
│   │   ├── right_angle_planner.py
│   │   ├── pure_pursuit_controller.py
│   │   ├── sim_sensor_bridge.py
│   │   ├── track_model.py
│   │   └── utils.py
│   ├── rviz/right_angle.rviz
│   └── urdf/right_angle_car.urdf.xacro
├── sim_perception/
│   └── sim_perception/
│       ├── sim_node.py
│       └── pyarmor_runtime_000000/
└── tracks/
    ├── models/
    │   ├── blue_cone/
    │   ├── yellow_cone/
    │   └── shixi/
    └── worlds/
        ├── right_angle_harmonic.sdf
        └── right_angle_wsl_headless.sdf
```

包功能：

- `fsd_common_msgs`：锥桶消息接口，包含 `Cone`、`ConeDetections`、`Map`。
- `right_angle_track`：赛道 world、锥桶模型、锥桶 mesh、赛道布置。
- `right_angle_stack`：车辆模型、launch、RViz、定位、建图、规划、控制。
- `sim_perception`：老师提供的加密感知包，保留原样。

## 系统数据流

核心话题如下：

| 层级 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 仿真控制输入 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制器输出，桥接到 Gazebo DiffDrive |
| 轮速里程计 | `/sensors/wheel_odom` | `nav_msgs/msg/Odometry` | Gazebo 车辆运动反馈 |
| GPS | `/sensors/gps/fix` | `sensor_msgs/msg/NavSatFix` | 经纬度定位输入 |
| IMU | `/sensors/imu/data_raw` | `sensor_msgs/msg/Imu` | 角速度、线加速度 |
| 磁力计 | `/sensors/magnetic_field` | `sensor_msgs/msg/MagneticField` | 航向辅助，当前默认不强融合 |
| 相机 | `/sensors/camera/image_raw` | `sensor_msgs/msg/Image` | RViz 可视化 |
| 相机内参 | `/sensors/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RViz/Image 工具使用 |
| 雷达扫描 | `/sensors/lidar/scan` | `sensor_msgs/msg/LaserScan` | RViz 可视化 |
| 雷达点云 | `/sensors/lidar/scan/points` | `sensor_msgs/msg/PointCloud2` | RViz 可视化 |
| 定位输出 | `/localization/pose` | `geometry_msgs/msg/PoseStamped` | 建图和感知使用 |
| 定位里程计 | `/localization/odom` | `nav_msgs/msg/Odometry` | 控制器使用 |
| 感知锥桶 | `/perception/cones` | `fsd_common_msgs/msg/Map` | `base_link` 下的局部锥桶 |
| 感知检测 | `/perception/cone_detections` | `fsd_common_msgs/msg/ConeDetections` | 单帧锥桶检测 |
| 建图输出 | `/estimation/slam/map` | `fsd_common_msgs/msg/Map` | `world` 下的锥桶地图 |
| 规划路径 | `/planning/centerline` | `nav_msgs/msg/Path` | 控制器跟踪路径 |
| 建图可视化 | `/visualization/cone_map` | `visualization_msgs/msg/MarkerArray` | RViz 锥桶 |
| 规划可视化 | `/visualization/planning` | `visualization_msgs/msg/MarkerArray` | RViz 中心线 |

当前总启动命令默认使用给的 sim_perception ，充当感知子系统。相机和雷达满足传感器建模与 RViz 可视化要求，当前不直接参与控制决策。

## 环境与构建

### 1. 清理旧进程

如果之前启动过 Gazebo、RViz 或 bridge，先清理。（重启大法也可以）

```bash
pkill -INT -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
sleep 2
pkill -9 -f "gz sim|ign gazebo|gazebo|gzserver|gzclient|rviz2|ros_gz_bridge|parameter_bridge|spawn_entity|create" || true
```

以上是在实体机调试时遇到的问题，gazebo每次启动时总会打开之前build出的界面，可能和终端退出不完全有关。
在WSL内同理。

### 2. 保证环境干净

调试时不要在同一个终端混ros2或工作空间，这样可能会命中上一次的版本，看不出后续改动的影响。
解决方案是新开终端后执行以下命令清洗工作环境：

```bash
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
```

### 3. 工作区位置

在WSL下，建议把工程放在 WSL ext4 文件系统，不要直接在/mnt/c,/mnt/e等目录里编译。由于文件权限问题，直接调用WSL外内容build容易失败。
实体机上基本无需考虑这个问题（真的方便）

### 4. 构建

```bash
cd ~/文档/SCUT_Racing_Tasks/homework/percep_node_track
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+ 
source install/setup.bash
```

## 启动流程

### 完整仿真：Gazebo GUI + RViz + 全部传感器

```bash
cd ~/文档/SCUT_Racing_Tasks/homework/percep_node_track
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf" 
```

### 一阶段分步调试

这部分是在WSL上进行的，最后开GUI时崩溃了（
由于WSL上图形栈不稳定，就先只查看各个节点发布话题信息是否准确（有变动未必是正常，要把噪音扰动考虑进来）

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=false \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

确认话题有数据后，在另一个终端单独打开 RViz（WSL上崩了）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/文档/SCUT_Racing_Tasks/homework/percep_node_track/install/setup.bash

rviz2 -d ~/文档/SCUT_Racing_Tasks/homework/percep_node_track/install/right_angle_stack/share/right_angle_stack/rviz/right_angle.rviz
```

向AI征求备用方案时，提到可先验证非渲染链路：

```bash
ros2 launch right_angle_stack right_angle_wsl_headless.launch.py use_rviz:=false
```

这个入口禁用相机和 GPU 雷达，只保留定位传感器、算法链路和控制闭环。它适合排查构建、spawn、bridge、定位、建图、规划和控制，不适合验证相机/雷达画面。

### 感知算法默认设置

默认启动时已经使用老师给的 sim_perception：

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

等价于默认参数：

```bash
use_builtin_perception:=false
use_sim_perception:=true
```

如果加密包运行环境临时有问题，可以切回内置 track_perception（AI生成，用于看数据细节） 做 fallback 调试：

```bash
ros2 launch right_angle_stack right_angle_harmonic.launch.py \
  use_builtin_perception:=true \
  use_sim_perception:=false \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

若 sim_perception 报缺少 pyarmor_runtime.so，检查

```bash
find sim_perception install/sim_perception -name 'pyarmor_runtime.so' -ls
```

该 .so 是运行加密包必需文件，不能被 .gitignore 屏蔽（先前误屏蔽了此文件，导致组员使用同步后的代码测试时失败）。

## 各环节开发

### 1. 仿真

仿真层的目标是先让 Gazebo 中存在可运动车辆、可见赛道、可桥接传感器数据，再考虑算法闭环。

主要文件：

- tracks/worlds/right_angle_harmonic.sdf
- tracks/models/shixi/model.sdf
- tracks/models/blue_cone/model.sdf
- tracks/models/yellow_cone/model.sdf
- right_angle_stack/models/right_angle_car_harmonic/model.sdf
- right_angle_stack/urdf/right_angle_car.urdf.xacro
- right_angle_stack/launch/right_angle_harmonic.launch.py

开发步骤：

1. 建立 world

   right_angle_harmonic.sdf中定义了right_angle_world、重力、磁场、地面、光照、spherical_coordinates和赛道模型。spherical_coordinates用来给GPS插件提供经纬度参考。

2. 建立赛道

   shixi/model.sdf 使用<include>布置蓝色和黄色锥桶。直道部分沿y轴，从y=-15到y=0；弯道后转到横向路径，形成右角弯。

3. 建立锥桶模型

   最终设计为：

   - visual 使用 .dae mesh，Gazebo 里外观就是锥桶。
   - collision 使用 cylinder，避免 mesh collision 在 DART/ODE 中带来不稳定。

   既满足可视化，又减少物理仿真碰撞问题。

4. 建立车辆模型

   right_angle_car_harmonic/model.sdf 中包含

   - base_link
   - 四个车轮 link 和 revolute joint
   - DiffDrive 插件
   - 相机 link 和 camera sensor
   - 雷达 link 和 gpu_lidar sensor
   - GPS、IMU、磁力计 sensor

   车辆通过 DiffDrive 接收 /cmd_vel，发布 /sensors/wheel_odom。

5. 建立 Gazebo/ROS 桥接

   right_angle_harmonic.launch.py 中使用 ros_gz_bridge parameter_bridge 桥接：

   - ROS -> Gazebo：/cmd_vel
   - Gazebo -> ROS：/clock、轮速里程计、GPS、IMU、磁力计、相机、雷达

6. 可视化

   Gazebo GUI 负责观察真实仿真场景；RViz 负责观察 ROS 侧数据，包括 TF、车辆模型、相机图像、雷达点云、锥桶地图和规划中心线。

### 2. 定位

定位节点：right_angle_stack/right_angle_stack/localization_fusion.py

输入：

- /sensors/gps/fix
- /sensors/imu/data_raw
- /sensors/wheel_odom
- /sensors/magnetic_field

输出：

```text
/localization/pose
/localization/odom
TF：world -> base_link
```

实现思路：

1. GPS 经纬度转局部米制坐标。

   使用局部切平面近似
   $x = R cos(lat_0) * (lon - lon_0)$
   $y = R (lat - lat_0)$
   其中 R = 6378137.0 m。

2. 第一帧 GPS 作为仿真参考点。

   调试中发现 Gazebo NavSat 插件输出和 world 中写死的经纬度参考可能存在偏差，直接用固定经纬度会把车拉到几万米外。所以当前实现让第一帧 GPS 映射到初始位姿，后续 GPS 再作为相对修正。

3. 轮速里程计提供前向速度。

   /sensors/wheel_odom.twist.twist.linear.x 作为车体前向速度。

4. IMU 提供 yaw rate。

   /sensors/imu/data_raw.angular_velocity.z 积分更新航向。

5. 磁力计当前默认关闭强融合。

   配置中 mag_gain: 0.0。因为仿真磁力计坐标和航向解释容易引入突变，先保证闭环稳定。代码中仍保留了磁力计融合逻辑和突变拒绝，后续可以标定后重新启用。

6. GPS突变应对。

   如果 GPS 转换后的位置距离当前融合位姿超过 gps_reject_distance，会拒绝该次 GPS 的数据，避免定位突然飞走，控制失效。

### 3. 感知输入

默认节点：sim_perception/sim_node.py

这里使用已给的 sim_perception。

### 4. 建图

节点：right_angle_stack/right_angle_stack/cone_mapper.py

输入

- /localization/pose
- /perception/cones
- /perception/cone_detections

输出

- /estimation/slam/map
- /visualization/cone_map

实现逻辑：

1. 接收局部感知锥桶。

   感知消息的 frame_id 通常是 base_link，表示锥桶坐标在车辆局部坐标系下。

2. 坐标变换。

   如果输入已经是 world 或 map，直接使用；如果是 base_link，根据 /localization/pose 做变换
   $$world_x = car_x + cos(yaw) * local_x - sin(yaw) * local_y \\
   world_y = car_y + sin(yaw) * local_x + cos(yaw) * local_y$$

3. 按颜色分类。

   借用ros2任务，用 blue、yellow、red、unknown 四类 bucket 保存 landmarks（虽然只有黄色和蓝色的锥桶）。

4. 地标融合。

   对同色锥桶，如果新观测和已有 landmark 距离小于 merge_distance，认为是同一个锥桶，用递推平均更新位置；否则创建新 landmark。

5. 地图发布。

   输出 fsd_common_msgs/msg/Map，坐标系为 world。

6. RViz 可视化。

   用 MarkerArray 发布 cylinder marker。这里 marker 用 cylinder 只是 RViz 显示，不影响 Gazebo 中真实锥桶 mesh。

开发中遇到的关键问题是定位飞走会直接导致建图为空。因为建图依赖 /localization/pose 把局部锥桶转换到 world，所以一旦定位不可信，感知范围和地图都会错。

### 5. 规划开发

节点：right_angle_stack/right_angle_stack/right_angle_planner.py

输入

- /estimation/slam/map
- /localization/pose

输出：

- /planning/centerline
- /visualization/planning

规划器有两套路径来源：

1. 锥桶地图中心线。

   如果 prefer_cone_map: true 且地图中有足够的蓝锥和黄锥：

   - 遍历蓝锥；
   - 找最近的黄锥配对；
   - 配对距离小于 pair_distance_max 才接受；
   - 取蓝黄锥中点作为中心线点；
   - 根据 track_progress() 排序；
   - 用 densify() 加密路径点。

2. 解析 fallback 路径。

   如果锥桶地图不足，使用解析路径保证车仍能完成直角弯：

   - 第一段：沿 x=0 从 y=-15 走到 y=0；
   - 第二段：以 (12, 0) 为圆心、半径 12 生成四分之一圆弯道；
   - 第三段：沿 y=12 向东直行。

路径输出为 nav_msgs/msg/Path，frame 为 world。RViz 中规划线颜色用于区分来源：

- cone map 路径：偏绿色；
- fallback 路径：白色。

调试时如果看到车一开始直行，这是正常现象，因为起点到弯道入口本来就是直道。只有接近 y=0 后才应该开始右转。

### 6. 控制开发

节点：right_angle_stack/right_angle_stack/pure_pursuit_controller.py

输入：

- /planning/centerline
- /localization/odom

输出：

- /cmd_vel

控制算法是 Pure Pursuit：

1. 从路径中找到距离当前车辆最近的点。
2. 从最近点往后找第一个满足距离大于 lookahead_distance 且在车辆前方的目标点。
3. 把目标点从 world 转到 base_link。
4. 计算曲率$curvature = 2\frac{local_y}{lookahead^2}$

5. 计算$yaw_rate = target_{speed}*curvature + yaw_{errorgain}*yaw_{error}$

6. 限制最大角速度 max_yaw_rate。
7. 根据曲率做简单降速，弯道中速度降低，直道恢复目标速度。
8. 发布 /cmd_vel.linear.x 和 /cmd_vel.angular.z。

如果 /cmd_vel.angular.z 已经非零但 Gazebo 里车不转，问题通常在 Gazebo 车辆模型或 /cmd_vel bridge；如果 /cmd_vel.angular.z 一直为零，问题通常在规划路径、车辆定位或 lookahead 目标点选择。

### 7. 闭环联调过程

联调时按下面顺序推进：

1. Gazebo 能启动，车辆和锥桶可见。
2. /cmd_vel 能通过 bridge 让 Gazebo 车辆运动。
3. GPS、IMU、轮速、磁力计 topic 有数据。
4. /localization/pose 在赛道附近，不飞到几万米外。
5. /perception/cones 在车辆前方有锥桶。
6. /estimation/slam/map 有 world 下的锥桶。
7. /planning/centerline 有完整右角弯路径。
8. /cmd_vel 有速度和角速度。
9. Gazebo 中车辆沿直道进入右角弯。
10. RViz 中能看到 TF、车辆模型、相机、雷达、锥桶地图和规划线。

## 调试流程

### 总体检查

```bash
ros2 node list
ros2 topic list
ros2 topic echo /clock --once
```

应该看到这些节点：

```text
/ros_gz_bridge
/robot_state_publisher
/localization_fusion
/sim_perception
/cone_mapper
/right_angle_planner
/pure_pursuit_controller
```

如果手动切换到 fallback，才会额外出现 /track_perception。

### 传感器检查

```bash
ros2 topic echo /sensors/wheel_odom --once
ros2 topic echo /sensors/imu/data_raw --once
ros2 topic echo /sensors/gps/fix --once
ros2 topic echo /sensors/magnetic_field --once
```

相机和雷达：

```bash
gz topic -l | grep -E 'camera|lidar|sensors'

ros2 topic list | grep -E 'camera|lidar'
ros2 topic echo /sensors/camera/camera_info --once
ros2 topic hz /sensors/camera/image_raw
ros2 topic echo /sensors/lidar/scan --once
ros2 topic echo /sensors/lidar/scan/points --once
```

### 不转向排查

```bash
ros2 topic echo /localization/pose --once
ros2 topic echo /perception/cones --once
ros2 topic echo /estimation/slam/map --once
ros2 topic echo /planning/centerline --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /sensors/wheel_odom --once
```

判断方式：

- /localization/pose 坐标离赛道很远：定位融合问题。
- /perception/cones 为空：感知范围、车辆位姿或锥桶 SDF 读取问题。
- /estimation/slam/map 为空：建图没有收到局部锥桶或定位不可用。
- /planning/centerline` 为空：规划器没有地图且 fallback 被关闭。
- /cmd_vel.angular.z 一直为 0：控制器没有看到弯道目标点，或定位/路径不一致。
- /cmd_vel.angular.z 非 0 但车不转：Gazebo DiffDrive 或 bridge 问题。

### 直接测试车辆驱动

绕过规划控制，直接发速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.4}}" -r 10
```

如果车能动并转向，车辆模型和 bridge 基本正常，问题在上游算法链路。

## 常见问题复盘

### fsd_common_msgs 在 Jazzy 下编译失败

现象：

```text
rosidl_generate_interfaces.cmake: list index: 1 out of range
Target dependency ... Cone.idl does not exist
```

原因：

Jazzy 的接口生成链路比旧版本更严格，原.msg适配成.idl时失败。

解决：

保留.msg方便理解架构，但 CMake 实际直接用相应的.idl（AI根据.msg编写的）

### CMakeCache 路径不一致

现象：

```text
The current CMakeCache.txt directory ... is different than the directory ...
```

原因：

把已经构建过的 workspace 从 Windows 路径复制到 WSL 路径时，build/ 里记录的绝对路径还是旧路径。

解决：

```bash
rm -rf build install log
colcon build --symlink-install
```

### Gazebo 找不到模型 mesh

现象：

```text
Unable to find file ... cone_blue.dae
Failed to load geometry for visual
```

解决：

- 确认 `tracks/models/blue_cone/meshes/cone_blue.dae` 存在。
- 确认 launch 设置了 `GZ_SIM_RESOURCE_PATH`。
- 锥桶 visual 使用 `model://blue_cone/meshes/cone_blue.dae`。
- collision 继续使用 cylinder。

### RViz 报 `frame [world] does not exist`

通常不是 RViz 的问题，而是 Gazebo 或定位节点没有正常跑起来。先检查：

```bash
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic echo /localization/pose --once
```

### 定位飞到几万米外

现象：

```text
/localization/pose:
x: -16866
y: 12862
```

原因：

GPS 经纬度原点和 Gazebo NavSat 输出没有对齐，或者磁力计航向突变参与融合。

解决：

- 第一帧 GPS 映射到初始位姿；
- GPS 突变拒绝；
- 磁力计默认 `mag_gain: 0.0`，后续标定后再启用。

## 组内协作

建议按模块分工：

- 环境：ROS/Gazebo 安装、WSL 图形栈、构建流程。
- 仿真：world、车辆、传感器、bridge、RViz。
- 定位：GPS/IMU/轮速/磁力计融合和 TF。
- 感知建图：锥桶观测、坐标变换、地图融合。
- 规划控制：中心线、Pure Pursuit、速度和角速度输出。
- 文档：记录问题、验证命令、解决方法、截图。

讨论问题时按数据链路定位，不按模块互相猜。例如“不转向”应该先问：

```text
cmd_vel 是否有角速度？
planning 是否有弯道？
localization 是否在赛道附近？
perception 是否看到锥桶？
Gazebo 是否收到 /cmd_vel？
```

这样比直接盯 Gazebo 画面更快。

## AI 使用情况

本工程中 AI 主要用于

- 阅读长日志，归类错误层级；
- 生成排查命令；
- 辅助修改 launch、SDF、RViz 和定位融合代码；
- 实现内置备份感知节点功能，区分于给的感知代码，仅用于调试；
- 整理 README。

## 答辩准备

建议能解释下面问题：

- 为什么 world 用 ENU，base_link 用 FLU？
- 为什么起点 (0, -15) 朝北对应 yaw pi/2？
- GPS 经纬度如何转局部米制坐标？
- 为什么第一帧 GPS 被用作仿真局部参考？
- 为什么磁力计当前默认不强融合？
- 为什么锥桶 visual 用 mesh，collision 用 cylinder？
- 为什么相机和雷达当前只用于可视化？
- 建图如何把 base_link 下锥桶变成 world 下地图？
- 规划如何从蓝黄锥生成中心线？
- fallback 路径为什么前半段是直行？
- Pure Pursuit 如何根据目标点算角速度？
- 如果车不转，按什么顺序排查？
