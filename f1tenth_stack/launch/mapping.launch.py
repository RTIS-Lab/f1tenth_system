"""
Map-building flow: ODOM EKF owns odom -> base_link, slam_toolbox builds the
pose graph and publishes map -> odom (its localization output during mapping).

Prereqs started separately:
  - bringup_launch.py  (vesc + urg/sick + /scan + base_link -> laser static TF +
                        /vesc_odom + /sensors/imu/raw)
  Teleop joy nodes are also in bringup_launch.py; you drive the car manually
  around the track while this file is running.

What this file starts:
  - ekf_filter_node     (vesc + imu -> /odometry/filtered + odom -> base_link,
                         publish_tf overridden to true)
  - slam_toolbox        online_async, mode: mapping, from
                         config/f1tenth_online_async.yaml
  - nav2_lifecycle_manager  to bring slam_toolbox to active state
  - static TFs          base_link -> vesc -> vesc_imu
                         (base_link -> laser is provided by bringup_launch.py)

tf chain while mapping: map -> odom -> base_link -> laser
                                             -> vesc -> vesc_imu

When the lap is done, save what each localization mode consumes.

For AMCL (grid map):
  ros2 service call /slam_toolbox/save_map \\
      slam_toolbox/srv/SaveMap "{name: {data: <basename>}}"
Two files are written to your CWD: <basename>.pgm and <basename>.yaml. Those
are what you later pass to fusion.launch.py localization_mode:=amcl
map:=/abs/path/to/<basename>.yaml for known-map localization.

For slam_toolbox localization (pose graph):
  ros2 service call /slam_toolbox/serialize_map \\
      slam_toolbox/srv/SerializePoseGraph "{filename: /abs/path/to/<basename>}"
Two files are written: <basename>.posegraph and <basename>.data. Those are
what fusion.launch.py localization_mode:=slam_toolbox loads; the .pgm/.yaml
from save_map cannot be used by that mode. Pass posegraph:=/abs/path/to/
<basename> (the name given to serialize_map). If the pose graph uses the same
basename as the grid map, you may instead reuse
map:=/abs/path/to/<basename>.yaml -- the launch file strips the extension and
slam_toolbox appends .posegraph/.data. Prefer an absolute filename so the
files land where you expect, not in the launch CWD.

Why slam_toolbox benefits from the EKF here:
  slam_toolbox is a pose-graph SLAM system. Its inputs are odom -> base
  (motion prior) and /scan (scan registration that corrects the prior).
  The quality of the motion prior directly affects scan-registration lock,
  especially on long straights (IMU yaw-rate damping) and under hard
  acceleration (IMU longitudinal-acceleration constraints reduce the
  impact of wheel slip). The EKF gives slam_toolbox a smoother, less
  drift-prone prior than raw /vesc_odom, which translates to cleaner
  loop closures in the final map.

Architecture note: this file uses the EKF's OWN odom -> base_link (Option B
in notes.txt). The alternative -- running rtis_lidar_odom too, so
scan-matching happens twice (rtis_lidar_odom then slam_toolbox) -- also
works but conceptually double-stacks lidar correction. This setup keeps
slam_toolbox as the sole scan-match owner, which matches the standard
REP-105 mapping pattern and matches what AMCL expects later in
localization_mode:=amcl.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')

    # --- Config paths ---
    odom_ekf_config_default = os.path.join(
        pkg_share, 'config', 'fusion.yaml')
    slam_config_default = os.path.join(
        pkg_share, 'config', 'f1tenth_online_async.yaml')

    # --- Launch args ---
    odom_ekf_config_la = DeclareLaunchArgument(
        'odom_ekf_config',
        default_value=odom_ekf_config_default,
        description='Config for the ODOM EKF (vesc + imu, world_frame=odom). '
                    'publish_tf is overridden to true below.'
    )
    slam_config_la = DeclareLaunchArgument(
        'slam_config',
        default_value=slam_config_default,
        description='Config for slam_toolbox online_async '
                    '(f1tenth_online_async.yaml, mode: mapping)'
    )

    ld = LaunchDescription([
        odom_ekf_config_la,
        slam_config_la,
        LogInfo(msg=['Mapping mode: ekf + slam_toolbox online_async']),
    ])

    # --- ODOM EKF (vesc + imu) -- owns odom -> base_link during mapping ---
    # publish_tf is overridden to true: the EKF owns odom -> base_link and
    # slam_toolbox publishes map -> odom. /odometry/filtered is published for
    # any consumer but slam_toolbox consumes odom -> base_link via tf, not
    # the topic.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('odom_ekf_config'),
            {'publish_tf': True},
        ],
    )

    # --- slam_toolbox online_async in mapping mode ---
    # Delegate node bring-up to slam_toolbox's own launch file so we pick
    # up its lifecycle wiring, parameter loading, and any future
    # slam_toolbox-internal changes. The required arg is slam_params_file.
    # slam_package is resolved as a literal here (not a LaunchConfiguration)
    # because get_package_share_directory expects a parse-time string; if you
    # ever need a different slam package name, edit this line directly.
    slam_pkg_share = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_config'),
            # slam_toolbox's online_async_launch.py defaults use_lifecycle_manager
            # and use_sim_time to false; we explicitly enable lifecycle here so
            # nav2_lifecycle_manager below can bring the node to active state.
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    # # --- Lifecycle manager for slam_toolbox (bring it to active) ---
    # # slam_toolbox is a lifecycle node; without activation it just sits
    # # uninitialized and never starts processing scans.
    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_mapping',
    #     output='screen',
    #     parameters=[{
    #         # slam_toolbox's online_async node name is 'slam_toolbox' by default.
    #         'node_names': ['slam_toolbox'],
    #         'autostart': True,
    #         'bond_timeout': 4.0,
    #     }],
    # )

    # --- Static TFs (always present, mirror fusion.launch.py) ---
    # base_link -> laser is intentionally NOT added here: bringup_launch.py
    # already publishes it, and a duplicate would cause a tf conflict.
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
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                   'vesc', 'vesc_imu'],
    )

    ld.add_action(ekf_node)
    ld.add_action(slam_launch)
    # ld.add_action(lifecycle_manager)
    ld.add_action(vesc_static_tf_node)
    ld.add_action(vesc_imu_static_tf_node)

    return ld
