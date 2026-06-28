from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fusion_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'fusion.yaml'
    )

    fusion_la = DeclareLaunchArgument(
        'fusion_config',
        default_value=fusion_config,
        description='Descriptions for sensor configs')

    ld = LaunchDescription([fusion_la])

    fusion_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('fusion_config')],
    )

    vesc_static_tf_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='static_baselink_to_vesc',
      arguments=['-0.10', '0.0', '0.06', '0.0', '0.0', '0.0', 'base_link', 'vesc']
    )
    vesc_imu_static_tf_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='vesc_to_imu',
      arguments=['0.0', '0.0', '0.0', '-1.57079633', '0.0', '3.14159', 'vesc', 'vesc_imu']
    )
    ld.add_action(fusion_node)
    ld.add_action(vesc_static_tf_node)
    ld.add_action(vesc_imu_static_tf_node)

    return ld
