import os
import yaml
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

## Function to modify gains of controller
def modify_gain(file_path,name_gains):
  
    # Read the current values from the file
    with open(file_path, 'r') as file:
        current_values = yaml.safe_load(file)

    # Print the current values
    print("Current values:")
    for key, value in current_values['OpID_Controller']['ros__parameters'].items():
        print(f"{key}: {value}")

    # Ask user if they want to change the values
    change_values = input("Do you want to change these values? (yes/no): ").lower()

    if change_values == "yes":
        # Receive input from the user
        Kp = float(input("Enter Kp value: "))
        Kd = float(input("Enter Kd value: "))

        # Define the YAML content
        yaml_content = {
            name_gains: {
                'ros__parameters': {
                    'Kp': Kp,
                    'Kd': Kd
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

## Definition of launch description
def generate_launch_description():
    
    iiwa_description = get_package_share_directory('iiwa_description')
    iiwa_description_share = get_package_prefix('iiwa_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    gains = os.path.join(
        get_package_share_directory('iiwa_description'),
        'config',
        'gains_id_controller.yaml',
    )
    name_file = "OpID_Controller"
    modify_gain(gains,name_file)
    print("Gains path:", gains)  

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        iiwa_description, 'urdf', 'kuka_iiwa.xacro'),
                                      description='Absolute path to robot urdf file'
    )
    model_path = os.path.join(iiwa_description, "models")
    model_path += pathsep + os.path.join(iiwa_description_share, "share")
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)
     
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    inititial_configuration = os.path.join(
        get_package_share_directory('iiwa_description'),
        'config',
        'initial_configuration.yaml',
    )
    
    init_pose = os.path.join(
        get_package_share_directory('iiwa_description'),
        'config',
        'initial_pose.yaml',
    )

    object_path = os.path.join(get_package_share_directory("iiwa_description"),
                                 "models",
                                 "cuboid_object", 
                                 "model.sdf")
    
    ext_camera_path = os.path.join(get_package_share_directory("iiwa_description"),
                                 "models",
                                 "external_camera", 
                                 "ext_camera.sdf"
    )
    #_______________________________________________________________________
    # Gazebo

    # Set the path to the world file
    world_file_name = 'custom_world.world'
    world_path = os.path.join(iiwa_description, 'worlds', world_file_name)
    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
        
    # Launch gazebo server and client
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),launch_arguments={'world': world}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    #_______________________________________________________________________
    # Robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[ '-entity', 'kuka_iiwa','-topic', 'robot_description',],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    #_______________________________________________________________________
    # Inverse Dynamics Control in Joint Space
    id_controller_node_spawn = Node (
        package='iiwa_description',
        executable='id_control_node',
        name='id_control_node',
        output='screen',
        parameters=[inititial_configuration]
    )
    #_______________________________________________________________________
    # Inverse Dynamics Control in Operational Space
    op_id_controller_node_spawn = Node (
        package='iiwa_description',
        executable='OpID_Controller',
        name='OpID_Controller',
        output='screen',
        parameters=[init_pose,gains]
    )
    # Planner in Operational Space
    op_planner_node_spawn = Node (
        package='iiwa_description',
        executable='Op_Planner',
        name='planner',
        output='screen',
    )
    #_______________________________________________________________________
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
    # Spawn external camera
    #spawn_external_camera = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #    arguments=[
    #        '-entity', 'external_camera',
    #        '-file', ext_camera_path,
    #        '-x', '0.0',  # X coordinate
    #        '-y', '0.0',  # Y coordinate
    #        '-z', '1.3',  # Z coordinate
    #        '-Y', '0.0',  # Yaw (rotation around Z axis)
    #       '-P', '-1.5708',  # Pitch (rotation around Y axis)
    #       '-R', '0.0'   # Roll (rotation around X axis)
    #   ],
    #   output='screen'
    #)

    return LaunchDescription([
        env_var,
        model_arg,
        declare_world_cmd,
        start_gazebo_server,
        start_gazebo_client,    
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=robot_state_publisher_node,
                                        on_start=[LogInfo(msg="______________ ROBOT SPAWN ______________"), 
                                                 spawn_robot])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=spawn_robot,
                                        on_start=[LogInfo(msg="______________JOINT STATE PUBLISHER SPAWN ______________"), 
                                                    joint_state_broadcaster_spawner])),  
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=joint_state_broadcaster_spawner,
                                        on_start=[LogInfo(msg="______________EFFORT CONTROLLER SPAWN ______________"), 
                                                    arm_controller_spawner])),  
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=arm_controller_spawner,
                                        on_exit=[LogInfo(msg="______________ ROBOT CONTROL SPAWN ______________"), 
                                                 op_id_controller_node_spawn])),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=op_id_controller_node_spawn,
                                        on_start=[LogInfo(msg="______________ PLANNER READY______________"), 
                                                 op_planner_node_spawn])),  
        #RegisterEventHandler(
        #    event_handler=OnProcessStart(target_action=op_planner_node_spawn,
        #                                on_start=[LogInfo(msg="______________ OBJECT SPAWN ______________"), 
        #                                        spawn_object])),
        ## Moveit
        # moveit_node = IncludeLaunchDescription(
        #            PythonLaunchDescriptionSource(
        #        os.path.join(get_package_share_directory('iiwa_description'),'launch/moveit.launch.py')
        #   ),
        #)
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('iiwa_description'),'launch/rviz.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('apriltag_ros'),'launch/tag_gazebo.launch.py')
            )
        ),
    ])
