cmake_minimum_required(VERSION 3.8)
project(real_experiments_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(iiwa_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(camera_stream src/camera_stream.cpp)
ament_target_dependencies(camera_stream
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
    OpenCV
    yaml-cpp
)

add_executable(tag1pose_node src/tag1pose.cpp)
ament_target_dependencies(tag1pose_node
    rclcpp
    std_msgs
    apriltag_msgs
    geometry_msgs
    tf2
)
add_executable(tag2pose_node src/tag2pose.cpp)
ament_target_dependencies(tag2pose_node
    rclcpp
    apriltag_msgs
    geometry_msgs
    tf2
)
add_executable(storage_data_node src/storage_data.cpp)
ament_target_dependencies(storage_data_node
    rclcpp
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

add_executable(real_data_monitor src/real_data_monitor.cpp)
ament_target_dependencies (real_data_monitor rclcpp std_msgs)

add_executable(ee_pose_and_vel_publisher src/realeeposepub.cpp)
ament_target_dependencies (ee_pose_and_vel_publisher rclcpp std_msgs)

add_executable(ftsensor_publisher src/realftsensor.cpp)
ament_target_dependencies (ftsensor_publisher rclcpp std_msgs)

add_executable(camera_publisher src/realcamera.cpp)
ament_target_dependencies (camera_publisher rclcpp std_msgs)

#_______________________________________________________________________________

install(TARGETS
  camera_stream
  real_data_monitor
  ee_pose_and_vel_publisher
  camera_publisher
  ftsensor_publisher
  tag1pose_node
  tag2pose_node
  storage_data_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
