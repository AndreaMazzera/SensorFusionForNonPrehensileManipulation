from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    Apriltag1pose_node = Node (
        package='iiwa_description',
        executable='tag1pose_node',
        output='screen'
    )
    Apriltag2pose_node = Node (
        package='iiwa_description',
        executable='tag2pose_node',
        output='screen'
    )
    
    return LaunchDescription([
        Apriltag1pose_node,
        Apriltag2pose_node
    ])
