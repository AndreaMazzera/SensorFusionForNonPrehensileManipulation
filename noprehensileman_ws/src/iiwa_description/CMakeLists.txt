cmake_minimum_required(VERSION 3.5)
project(iiwa_description)

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

# __________________________________________________________________
# ________________________JOINT SPACE_______________________________
add_executable(joint_space_planner_node src/joint_space_planner.cpp)
ament_target_dependencies (joint_space_planner_node 
  rclcpp 
  std_msgs
  geometry_msgs 
  sensor_msgs 
  trajectory_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
)

add_executable(id_control_node src/joint_space_id_control.cpp)
ament_target_dependencies(id_control_node
  rclcpp
  std_msgs
  sensor_msgs
  kdl_parser
  trajectory_msgs
)

# ________________________________________________________________________
# ________________________OPERATIONAL SPACE_______________________________
add_executable(Op_Planner src/operational_space_planner.cpp)
ament_target_dependencies (Op_Planner 
  rclcpp 
  std_msgs
  geometry_msgs 
  sensor_msgs 
  trajectory_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
  iiwa_msgs
  visualization_msgs
)

add_executable(OpID_Controller src/operational_space_id_control.cpp)
ament_target_dependencies (OpID_Controller 
  rclcpp  
  std_msgs 
  geometry_msgs
  sensor_msgs 
  kdl_parser
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  iiwa_msgs
)

add_executable(service_client_node src/client.cpp)
ament_target_dependencies (service_client_node rclcpp iiwa_msgs Eigen3 geometry_msgs)

#_______________________________________________________________________________
#_____________________________Utility___________________________________________

# Obtain and storage data in CSV files
add_executable(monitor src/monitor.cpp)
ament_target_dependencies (monitor rclcpp std_msgs sensor_msgs)

#_______________________________________________________________________________
#______________FUTURE VERSIONS OF CLIENT AND PLANNER____________________________

add_executable(new_client src/new_client.cpp)
ament_target_dependencies (new_client rclcpp std_msgs Eigen3)

add_executable(new_planner src/new_planner.cpp)
ament_target_dependencies (new_planner rclcpp std_msgs Eigen3 tf2 moveit_msgs)
#_______________________________________________________________________________


install(TARGETS
  joint_space_planner_node
  id_control_node
  OpID_Controller
  Op_Planner
  service_client_node
  monitor
  new_client
  new_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch meshes models rviz src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()