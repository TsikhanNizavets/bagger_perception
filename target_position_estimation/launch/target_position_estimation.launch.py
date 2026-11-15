from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'target_position_estimation'

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    show_point_cloud = LaunchConfiguration('show_point_cloud')
    declare_show_point_cloud_cmd = DeclareLaunchArgument(
        'show_point_cloud',
        default_value='false',
        description='Whether to show point cloud',
    )

    target_position_estimation_node = Node(             
        package=package_name,
        executable='target_position_estimation',
        name='target_position_estimation',
        parameters=[{'use_sim_time': use_sim_time, 'show_point_cloud': show_point_cloud}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_show_point_cloud_cmd,
        target_position_estimation_node,
    ])
