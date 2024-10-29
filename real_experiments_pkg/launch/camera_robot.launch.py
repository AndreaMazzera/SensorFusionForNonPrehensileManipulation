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

def generate_launch_description():
    
    iiwa_description = get_package_share_directory('iiwa_description')
    iiwa_description_share = get_package_prefix('iiwa_description')
    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        iiwa_description, 'urdf', 'robot_camera.urdf'),
                                      description='Absolute path to robot urdf file'
    )
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
        # RViz
    rviz_config = os.path.join(
        get_package_share_directory("iiwa_description"),
            "rviz",
            "test.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{'use_sim_time': True}],  # Adjust parameters as needed
        arguments=["-d", rviz_config],
    )
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('apriltag_ros'),'launch/tag_gazebo.launch.py')
            )
        ),
    ])
