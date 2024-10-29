import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def modify_gain(file_path):
  
    # Read the current values from the file
    with open(file_path, 'r') as file:
        current_values = yaml.safe_load(file)

    # Print the current values
    print("Current values:")
    for key, value in current_values['ExtendendKalmanFilter']['ros__parameters'].items():
        print(f"{key}: {value}")

    # Ask user if they want to change the values
    change_values = input("Do you want to change these values? (yes/no): ").lower()

    if change_values == "yes":
        # Receive input from the user
        Kq = float(input("Enter Kq value: "))
        Kr = float(input("Enter Kr value: "))
        Kp = float(input("Enter Kp value: "))

        # Define the YAML content
        yaml_content = {
            'ExtendendKalmanFilter': {
                'ros__parameters': {
                    'Kq': Kq,
                    'Kr': Kr,
                    'Kp': Kp
                }
            }
        }

        # Write the YAML content to the file
        with open(file_path, 'w') as file:
            yaml.dump(yaml_content, file)

        print("File saved successfully.")
    elif change_values == "no":
        print("Values remain unchanged.")
    else:
        print("Invalid input. Values remain unchanged.")



def generate_launch_description():
    
    # The user choose version of EKF with or without motion model of object
    input_valid = False
    while not input_valid:
        
        print("Choose the ekf implementation:")
        print("- A: ekf with model")
        print("- B: ekf without model")
        user_input = input().strip()

        if user_input.upper() == 'A':
            selected_node = 'ekf_with_model'
            gains_ekf_path = os.path.join(
                get_package_share_directory('ekf_pkg'),
                'config',
                'gains_ekf_with_model.yaml',
            )
            modify_gain(gains_ekf_path)
            input_valid = True
        elif user_input.upper() == 'B':
            selected_node = 'ekf_without_model'
            gains_ekf_path = os.path.join(
                get_package_share_directory('ekf_pkg'),
                'config',
                'gains_ekf_without_model.yaml',
            )
            modify_gain(gains_ekf_path)
            input_valid = True
        else:
            print("Invalid choice. Please enter 'A' or 'B'.")

    # Print the gains path
    print("Gains path:", gains_ekf_path)  

    ekf_namespace = "ExtendedKalmanFilter"

    object_path = os.path.join(get_package_share_directory("iiwa_description"),
                                 "models",
                                 "cuboid_object", 
                                 "model.sdf")

    # Spawn object
    spawn_object = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'CUBOID',
            '-file', object_path,
            '-x', '0.25',  # X coordinate
            '-y', '0.0',  # Y coordinate
            '-z', '0.95',  # Z coordinate
            '-Y', '0.0',  # Yaw (rotation around Z axis)-0.230568
            '-P', '0.0',  # Pitch (rotation around Y axis)-1.424919
            '-R', '0.0'   # Roll (rotation around X axis)-2.825518
        ],
        output='screen'
    )
    #_______________________________________________________________________
    # ___________ NODES FOR COORDINATES CONVERSION___________#
    real_object_pose_node = Node(
        package='ekf_pkg',
        executable='RealPoseObj_PubNode',
        namespace=ekf_namespace, 
        output='screen',
    )
    
    ftsensor_object_pose_node = Node(
        package='ekf_pkg',
        executable='FTSensor_PoseObj_PubNode',
        namespace=ekf_namespace, 
        output='screen',
    )

    camera_object_pose_node = Node(
        package='ekf_pkg',
        executable='CamSensor_PoseObj_PubNode',
        namespace=ekf_namespace, 
        output='screen',
    )
    # _______________________________________________________#

    ekf_node = Node(
        package='ekf_pkg',
        executable=selected_node,
        name='ExtendendKalmanFilter',
        output='screen',
        namespace=ekf_namespace, 
        parameters=[gains_ekf_path]
    )

    error_values_node = Node (
        package='ekf_pkg',
        executable='ObjPoseError_PubNode',
        namespace=ekf_namespace, 
        output='screen',
    )

    obj_respawner_node = Node (
        package='ekf_pkg',
        executable='ObjRespawner_Node',
        namespace=ekf_namespace,
        output='screen',
    )

    return LaunchDescription([
        spawn_object,
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=spawn_object,
                                        on_start=[LogInfo(msg="_____________Real Pose Ready______________"), 
                                                real_object_pose_node])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=real_object_pose_node,
                                        on_start=[LogInfo(msg="_____________FT Sensor Ready______________"), 
                                                ftsensor_object_pose_node])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=ftsensor_object_pose_node,
                                        on_start=[LogInfo(msg="_____________Camera Ready______________"), 
                                                camera_object_pose_node])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=camera_object_pose_node,
                                        on_start=[LogInfo(msg="_____________EFK Ready______________"), 
                                                ekf_node])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=ekf_node,
                                        on_start=[LogInfo(msg="_____________Computing Errors______________"), 
                                                error_values_node])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=error_values_node,
                                        on_start=[LogInfo(msg="_____________Object Respawner Ready______________"), 
                                                obj_respawner_node])),    
    ])
