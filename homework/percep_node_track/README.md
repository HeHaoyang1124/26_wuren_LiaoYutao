## right_angle_stack 仿真执行逻辑

### 整体架构

整个仿真由 **Gazebo Harmonic** 物理引擎驱动，车辆在 `right_angle_harmonic.sdf` 定义的直角转弯赛道上行驶。
以下按数据流顺序描述各节点的功能与协作关系。

### 节点执行流水线

```
┌────────────────────────────────────────────────────────────┐
│  Gazebo Harmonic 仿真世界                                    
│  (world: right_angle_harmonic.sdf, 含赛道、锥桶、车辆模型)    
│                                                             
│  ┌─────────────────────────────────────────────────┐       
│  │ 传感器模型                                              
│  │ GPS / IMU / 磁力计 / 轮速计 / 激光雷达 / 相机            
│  └──────┬──────────────────────────────────────────┘       
└─────────┼──────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────┐
│ ros_gz_bridge (参数桥接)          
│ 将 Gazebo 传感器数据桥接到 ROS2   
│ 输出话题:                         
│   /sensors/gps/fix            
│   /sensors/imu/data_raw       
│   /sensors/magnetic_field     
│   /sensors/wheel_odom         
│   /sensors/lidar/scan         
│   /sensors/lidar/scan/points  
│   /sensors/camera/image_raw   
│   /sensors/camera/camera_info 
│ 反向桥接: /cmd_vel → Gazebo       
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│ ① localization_fusion         
│ 融合 GPS + IMU 陀螺 + 轮速计     
│ + 磁力计 进行定位                 
│                               
│ 算法:                       
│ - GPS: 经纬度转ENU坐标,          
│   低通滤波融合位置 (gps_gain)    
│ - IMU: 提供角速度 (yaw_rate) 
│ - 轮速计: 提供前进速度            
│ - 磁力计: 修正航向角 (mag_gain)  
│ - 死推算: 每20ms用速度+角速度积分  
│                               
│ 输出:                       
│   /localization/pose (位姿)      
│   /localization/odom (里程计)   
│   TF: world → base_link         
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│ ② track_perception              
│ 赛道感知（模拟感知模块）          
│                     
│ 从 SDF 文件中加载全部锥桶的    
│ 世界坐标, 根据当前车辆位置      
│ 将视野内的锥桶转换到 base_link
│ 坐标系, 并加入位置噪声模拟感知    
│                                  
│ 输出:                       
│   /perception/cones (Map,        
│     base_link 坐标系)            
│   /perception/cone_detections     
│     (ConeDetections,             
│     base_link 坐标系)            
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│ ③ cone_mapper                   
│ 锥桶建图                          
│                                  
│ 将 base_link 系下的局部锥桶       
│ 变换到 world 全局坐标系           
│ 对邻近锥桶进行合并去重            
│ (merge_distance 阈值内视为同一个) 
│                                  
│ 输出:                            
│   /estimation/slam/map (Map,        
│     world 坐标系)                
│   /visualization/cone_map        
│     (MarkerArray, RViz可视化)    
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│ ④ right_angle_planner           
│ 直角转弯路径规划                  
│                                  
│ 两种模式:                        
│ 1) 锥桶中心线: 配对蓝/黄锥桶,    
│    取中点并排序生成中心线路径     
│ 2) 解析路径 (fallback):          
│    直行→弧形转弯→直行的硬编码路径 
│                                  
│ 输出:                            
│   /planning/centerline (Path)      
│   /visualization/planning        
│     (MarkerArray, RViz可视化)    
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│ ⑤ pure_pursuit_controller       
│ 纯跟踪控制器                      
│                                  
│ 在规划路径上找前视距离             
│ (lookahead_distance) 外的目标点, 
│ 根据纯跟踪算法计算曲率→横摆角速度 
│ 弯道自动减速 (turn_slowdown)      
│ 抵达终点附近时停车                
│                                  
│ 输出:                            
│   /cmd_vel (Twist)              
│      → ros_gz_bridge → Gazebo   
└──────────────────────────────────┘
```

### 各节点职责总结

| 节点                        | 包                     | 功能                 | 关键输入                                                                                          | 关键输出                                               |
|---------------------------|-----------------------|--------------------|-----------------------------------------------------------------------------------------------|----------------------------------------------------|
| `localization_fusion`     | right_angle_stack_cpp | GPS/IMU/轮速/磁力计融合定位 | `/sensors/gps/fix`, `/sensors/imu/data_raw`, `/sensors/wheel_odom`, `/sensors/magnetic_field` | `/localization/pose`, `/localization/odom`, TF     |
| `track_perception`        | right_angle_stack_cpp | 模拟赛道锥桶感知           | `/localization/pose`, SDF赛道模型                                                                 | `/perception/cones`, `/perception/cone_detections` |
| `cone_mapper`             | right_angle_stack_cpp | 局部锥桶→全局地图+合并去重     | `/perception/cones`, `/perception/cone_detections`, `/localization/pose`                      | `/estimation/slam/map`, `/visualization/cone_map`  |
| `right_angle_planner`     | right_angle_stack_cpp | 生成直角转弯中心线路径        | `/estimation/slam/map`                                                                        | `/planning/centerline`                             |
| `pure_pursuit_controller` | right_angle_stack_cpp | 纯跟踪横向+纵向控制         | `/planning/centerline`, `/localization/odom`                                                  | `/cmd_vel`                                         |

### 启动命令

```bash
ros2 launch right_angle_stack_cpp right_angle_harmonic.launch.py \
  use_rviz:=true \
  gz_args:="-r -v 4 $(ros2 pkg prefix right_angle_track)/share/right_angle_track/worlds/right_angle_harmonic.sdf"
```

### 编译命令

```bash
cd <workspace_root>
colcon build --packages-select right_angle_stack_cpp
source install/setup.bash
```