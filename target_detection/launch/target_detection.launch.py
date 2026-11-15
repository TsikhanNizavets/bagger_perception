from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'target_detection'

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    show_yoloe_detections = LaunchConfiguration('show_yoloe_detections')
    declare_show_yoloe_detections_cmd = DeclareLaunchArgument(
        'show_yoloe_detections',
        default_value='false',
        description='Whether to show YOLOE detections',
    )

    target_detection_node = Node(             
        package=package_name,
        executable='target_detection',
        name='target_detection',
        parameters=[{
            'use_sim_time': use_sim_time,
            'show_yoloe_detections': show_yoloe_detections,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_show_yoloe_detections_cmd,
        target_detection_node,
    ])
