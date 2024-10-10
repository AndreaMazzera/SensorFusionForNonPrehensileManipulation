import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    ee_pose_and_vel_publisher = Node(
        package='iiwa_description',
        executable='ee_pose_and_vel_publisher',
        name='ee_pose_and_vel_publisher'
    )

    ftsensor_publisher = Node(
        package='iiwa_description',
        executable='ftsensor_publisher',
        name='ftsensor_publisher'
    )

    camera_publisher = Node(
        package='iiwa_description',
        executable='camera_publisher',
        name='camera_publisher'
    )
    
    start_camera_publisher = RegisterEventHandler(
        OnProcessStart(
            target_action=ee_pose_and_vel_publisher,
            on_start=[camera_publisher]
        )
    )

    start_ftsensor_publisher = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_publisher,
            on_start=[ftsensor_publisher]
        )
    )

    

    return LaunchDescription([
        ee_pose_and_vel_publisher,
        start_camera_publisher,
        start_ftsensor_publisher,
        LogInfo(msg="Launch file initialized successfully."),
    ])
