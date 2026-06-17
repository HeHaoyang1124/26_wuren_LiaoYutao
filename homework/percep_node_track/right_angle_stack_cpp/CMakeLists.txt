cmake_minimum_required(VERSION 3.8)
project(right_angle_stack_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fsd_common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(localization_fusion src/localization_fusion.cpp)
ament_target_dependencies(localization_fusion
  rclcpp fsd_common_msgs geometry_msgs nav_msgs
  sensor_msgs std_msgs tf2_ros tf2 visualization_msgs)

add_executable(track_perception src/track_perception.cpp src/track_model.cpp)
ament_target_dependencies(track_perception
  rclcpp fsd_common_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs)

add_executable(cone_mapper src/cone_mapper.cpp)
ament_target_dependencies(cone_mapper
  rclcpp fsd_common_msgs geometry_msgs visualization_msgs
  std_msgs tf2)

add_executable(right_angle_planner src/right_angle_planner.cpp)
ament_target_dependencies(right_angle_planner
  rclcpp fsd_common_msgs geometry_msgs nav_msgs
  visualization_msgs std_msgs)

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)

install(TARGETS
  localization_fusion
  track_perception
  cone_mapper
  right_angle_planner
  pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config models rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()