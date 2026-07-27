from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')

    # --- ODOM EKF (vesc_odom + imu) ---
    # Runs in world_frame=odom and emits /odometry/filtered. publish_tf is
    # False: rtis_lidar_odom owns odom -> base_link (its 'real-world' mode,
    # pub_tf:=true) after consuming /odometry/filtered as a motion prior. No
    # map->odom transform is published in this architecture.
    odom_ekf_config = os.path.join(pkg_share, 'config', 'fusion.yaml')

    # Archived 2-EKF design that published map -> odom. Disabled by default; see
    # config/fusion_map.yaml header for why it does not correct drift.
    map_ekf_config = os.path.join(pkg_share, 'config', 'fusion_map.yaml')

    odom_ekf_config_la = DeclareLaunchArgument(
        'odom_ekf_config',
        default_value=odom_ekf_config,
        description='Config for the ODOM EKF (vesc + imu, world_frame=odom)'
    )
    map_ekf_config_la = DeclareLaunchArgument(
        'map_ekf_config',
        default_value=map_ekf_config,
        description='Config for the MAP EKF (lidar only, world_frame=map)'
    )
    # Default to OFF: the 2-EKF MAP layer was found to not actually correct
    # drift (see fusion_map.yaml header). Set enable_map_ekf:=true only for
    # debugging the legacy behaviour.
    enable_map_ekf_la = DeclareLaunchArgument(
        'enable_map_ekf',
        default_value='false',
        description='Launch the archived MAP EKF (default off)'
    )

    ld = LaunchDescription([odom_ekf_config_la, map_ekf_config_la, enable_map_ekf_la])

    odom_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('odom_ekf_config')],
    )

    map_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[LaunchConfiguration('map_ekf_config')],
        condition=IfCondition(LaunchConfiguration('enable_map_ekf')),
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
    ld.add_action(odom_ekf_node)
    ld.add_action(map_ekf_node)
    ld.add_action(vesc_static_tf_node)
    ld.add_action(vesc_imu_static_tf_node)

    return ld
