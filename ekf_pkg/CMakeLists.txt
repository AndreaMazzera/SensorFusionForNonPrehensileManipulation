cmake_minimum_required(VERSION 3.8)
project(ekf_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
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
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)


# ___________________________________________________________________
# ____________________________EKF Versions___________________________

add_executable(ekf_without_model src/ekf_without_model.cpp)
ament_target_dependencies(ekf_without_model rclcpp geometry_msgs)

add_executable(ekf_with_model src/ekf_with_model.cpp)
ament_target_dependencies(ekf_with_model rclcpp geometry_msgs Eigen3)

# ___________________________________________________________________
# _________________Nodes for coordinates conversion__________________

add_executable(RealPoseObj_PubNode src/obj_pose_real.cpp)
ament_target_dependencies(RealPoseObj_PubNode
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  nav_msgs
  gazebo_msgs
  visualization_msgs
)

add_executable(FTSensor_PoseObj_PubNode src/obj_pose_ftsensor.cpp)
ament_target_dependencies(FTSensor_PoseObj_PubNode
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(CamSensor_PoseObj_PubNode src/obj_pose_camera.cpp)
ament_target_dependencies(CamSensor_PoseObj_PubNode
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
  Eigen3
)

# ___________________________________________________________________
# ______________Node to compute object pose error____________________
add_executable(ObjPoseError_PubNode src/obj_pose_error.cpp)
ament_target_dependencies(ObjPoseError_PubNode 
  rclcpp 
  std_msgs 
  geometry_msgs
)

#_______________________________________________________________________________
#_____________________________Utility___________________________________________

# Respawn object in case of falling
add_executable(ObjRespawner_Node src/obj_respawn.cpp)
ament_target_dependencies (ObjRespawner_Node rclcpp std_msgs gazebo_msgs)
#_______________________________________________________________________________



install(TARGETS
  CamSensor_PoseObj_PubNode
  ekf_with_model
  ekf_without_model
  ObjPoseError_PubNode
  FTSensor_PoseObj_PubNode
  RealPoseObj_PubNode
  ObjRespawner_Node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch src
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
