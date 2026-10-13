"""
Sensor fusion + localization launch.

Two flows are supported via the `localization_mode` launch arg. Both start the
ODOM EKF (vesc + imu); they differ in who owns odom -> base_link and where the
global (map) frame comes from. AMCL when enabled is brought up *in this same
launch file* so users have one command to start the full stack.

  localization_mode:=lidar_odom  (default, no map frame)
    ekf_filter_node              /odometry/filtered (publish_tf: false)
    rtis_lidar_odom (started separately in the drive container)
                                 owns odom -> base_link + /lidar_odom
    tf:  odom -> base_link -> vesc -> vesc_imu
    Use case: drift-bounded odometry when no known map is available, or when
    you need a high-rate, locally-smooth odom -> base_link.

  localization_mode:=amcl  (requires a known static map)
    ekf_filter_node              /odometry/filtered + OWN odom -> base_link  (publish_tf: true)
    nav2_map_server              latched /map
    nav2_amcl                    owns map -> odom (reads /scan + odom -> base_link)
    nav2_lifecycle_manager       activates the above
    tf:  map -> odom -> base_link -> vesc -> vesc_imu
    Use case: racing on a known track; AMCL gives you global localization.
    You do NOT start rtis_lidar_odom in this mode -- the EKF's dead-reckoned
    odom -> base_link is the motion model AMCL uses for particle propagation
    between scans, and the lidar correction manifests as map -> odom.

Both modes inherit the vesc -> base_link and vesc -> vesc_imu static
transforms from this file.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    NotSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')

    odom_ekf_config_default = os.path.join(pkg_share, 'config', 'fusion.yaml')
    amcl_config_default = os.path.join(pkg_share, 'config', 'amcl.yaml')

    # --- Shared launch args ---
    odom_ekf_config_la = DeclareLaunchArgument(
        'odom_ekf_config',
        default_value=odom_ekf_config_default,
        description='Config for the ODOM EKF (vesc + imu, world_frame=odom)'
    )

    # `localization_mode` selects between the two flows described above.
    # `amcl` here is shorthand for "amcl-instead-of-rtis_lidar_odom"; the
    # lidar odom node is not started by this launch file in either mode --
    # in lidar_odom mode you start it separately in the drive container.
    localization_mode_la = DeclareLaunchArgument(
        'localization_mode',
        default_value='lidar_odom',
        description='lidar_odom (default, drift-bounded odom, no map) | amcl '
                    '(known-map localization; requires map:=<path>')
    is_amcl = PythonExpression([
        '"', LaunchConfiguration('localization_mode'), '" == "amcl"'
    ])
    is_lidar_odom = NotSubstitution(is_amcl)

    # --- AMCL-specific args (only used when localization_mode:=amcl) ---
    map_la = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Absolute path to the map .yaml for AMCL mode (required '
                    'when localization_mode:=amcl)'
    )
    amcl_config_la = DeclareLaunchArgument(
        'amcl_config',
        default_value=amcl_config_default,
        description='Config yaml for nav2_amcl + map_server (amcl.yaml)'
    )
    use_map_server_la = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Launch nav2_map_server. Set to false to reuse an '
                    'externally-provided /map.'
    )

    ld = LaunchDescription([
        odom_ekf_config_la,
        localization_mode_la,
        map_la,
        amcl_config_la,
        use_map_server_la,
        # Sanity log so users see which mode is active.
        LogInfo(msg=['Localization mode: ', LaunchConfiguration('localization_mode')]),
    ])

    # --- ODOM EKF (vesc + imu) -- always running ---
    # In lidar_odom mode: publish_tf stays false (per fusion.yaml). The lidar
    # odom node owns odom -> base_link. In amcl mode: override publish_tf to
    # true so the EKF owns odom -> base_link itself and AMCL has a motion
    # model to propagate between scans.
    odom_ekf_node_lidar_odom_mode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('odom_ekf_config')],
        condition=IfCondition(is_lidar_odom),
    )
    odom_ekf_node_amcl_mode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('odom_ekf_config'),
            {'publish_tf': True},
        ],
        condition=IfCondition(is_amcl),
    )

    # --- AMCL stack (only when localization_mode:=amcl) ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('amcl_config'),
            {'yaml_filename': LaunchConfiguration('map')},
        ],
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('use_map_server'), '" == "true" and ',
                '"', LaunchConfiguration('localization_mode'), '" == "amcl"',
            ])
        ),
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_config')],
        condition=IfCondition(is_amcl),
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'node_names': ['amcl', 'map_server'],
            'autostart': True,
            'bond_timeout': 4.0,
        }],
        condition=IfCondition(is_amcl),
    )

    # --- Static transforms (always present) ---
    vesc_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_vesc',
        arguments=['-0.10', '0.0', '0.06', '0.0', '0.0', '0.0',
                   'base_link', 'vesc'],
    )
    vesc_imu_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vesc_to_imu',
        # x y z roll pitch yaw
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                   'vesc', 'vesc_imu'],
    )

    ld.add_action(odom_ekf_node_lidar_odom_mode)
    ld.add_action(odom_ekf_node_amcl_mode)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(vesc_static_tf_node)
    ld.add_action(vesc_imu_static_tf_node)

    return ld
