import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('target_position_estimation'),
        'worlds',
        'plane.sdf',
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rqbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rqbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rqbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rqbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    'gz_args': f'-r "{world_path}"',
                }.items(),
            )

    return LaunchDescription([
        bridge,
        gazebo,
    ])