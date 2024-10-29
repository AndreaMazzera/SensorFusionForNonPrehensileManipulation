import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("iiwa_description"),
            "rviz",
            "kuka_control.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{'use_sim_time': True}],  # Adjust parameters as needed
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([rviz_node])
