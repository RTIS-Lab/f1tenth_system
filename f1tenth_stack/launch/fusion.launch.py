r"""
Sensor fusion + localization launch.

Three flows are supported via the `localization_mode` launch arg. All of them
start the ODOM EKF (vesc + imu); they differ in who owns odom -> base_link and
where the global (map) frame comes from. The map-based localizers are brought
up *in this same launch file* so users have one command to start the full stack.

  localization_mode:=lidar_odom  (default, no map frame)
    ekf_filter_node              /odometry/filtered (publish_tf: false)
    rtis_lidar_odom (started separately in the drive container)
                                 owns odom -> base_link + /lidar_odom
    tf:  odom -> base_link -> vesc -> vesc_imu
    Use case: drift-bounded odometry when no known map is available, or when
    you need a high-rate, locally-smooth odom -> base_link.

  localization_mode:=amcl  (requires a known static map)
    ekf_filter_node              /odometry/filtered + OWN odom -> base_link
                                 (publish_tf overridden to true)
    nav2_map_server              latched /map
    nav2_amcl                    owns map -> odom (reads /scan + odom -> base_link)
    nav2_lifecycle_manager       activates the above
    tf:  map -> odom -> base_link -> vesc -> vesc_imu
    Use case: racing on a known track; AMCL gives you global localization.
    You do NOT start rtis_lidar_odom in this mode -- the EKF's dead-reckoned
    odom -> base_link is the motion model AMCL uses for particle propagation
    between scans, and the lidar correction manifests as map -> odom.

  localization_mode:=slam_toolbox  (requires a serialized pose graph)
    ekf_filter_node              /odometry/filtered + OWN odom -> base_link
                                 (publish_tf overridden to true)
    slam_toolbox (localization)  latched /map + owns map -> odom
                                 loads <posegraph>.posegraph + <posegraph>.data
    tf:  map -> odom -> base_link -> vesc -> vesc_imu
    Use case: known track, elastic pose-graph localization. slam_toolbox
    scan-matches against the saved pose graph instead of a grid map, which can
    be more robust than AMCL on repetitive or feature-poor tracks, at the cost
    of needing good odometry and a serialized map (save_map's .pgm/.yaml is
    not enough). Create the pose graph while mapping with:
      ros2 service call /slam_toolbox/serialize_map \
          slam_toolbox/srv/SerializePoseGraph "{filename: /abs/path/to/track}"
    Then pass posegraph:=/abs/path/to/track, where `track` is the basename
    slam_toolbox serialized to (without .posegraph/.data). If the pose graph
    shares the grid map's basename, you can instead reuse the
    map:=/abs/path/to/track.yaml you use for AMCL -- the extension is
    stripped and slam_toolbox appends .posegraph/.data.
    You do NOT start rtis_lidar_odom in this mode either.

All modes inherit the vesc -> base_link and vesc -> vesc_imu static transforms
from this file.
"""

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    NotSubstitution,
    OrSubstitution,
    PythonExpression,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


_POSE_GRAPH_SUFFIXES = ('.posegraph', '.data', '.yaml', '.yml', '.pgm', '.png')


class PoseGraphBasename(Substitution):
    """
    Resolve slam_toolbox's map_file_name from the posegraph/map launch args.

    slam_toolbox appends .posegraph and .data to map_file_name, so callers
    normally have to pass a bare basename. Accepting the same .yaml path AMCL
    uses (or a path that still carries .posegraph) keeps the CLI symmetric.
    """

    def __init__(self, posegraph, map_path):
        super().__init__()
        self.__posegraph = normalize_to_list_of_substitutions(posegraph)
        self.__map_path = normalize_to_list_of_substitutions(map_path)

    def perform(self, context):
        path = (
            perform_substitutions(context, self.__posegraph) or
            perform_substitutions(context, self.__map_path)
        )
        for suffix in _POSE_GRAPH_SUFFIXES:
            if path.endswith(suffix):
                path = path[:-len(suffix)]
                break
        if not path:
            raise RuntimeError(
                'localization_mode:=slam_toolbox requires posegraph:=<basename> '
                'or map:=<path>.yaml. slam_toolbox localization loads a '
                'serialized pose graph (.posegraph + .data), not a .pgm/.yaml '
                'map. Create it while mapping with the '
                'slam_toolbox/serialize_map service.')
        if not os.path.isfile(path + '.posegraph'):
            hint = ''
            if os.path.isfile(path + '.s_t.posegraph'):
                hint = ' Did you mean posegraph:={0}.s_t?'.format(path)
            raise RuntimeError(
                "No pose graph found at '{0}.posegraph'. slam_toolbox "
                "localization loads a serialized pose graph, not a .pgm/.yaml "
                "map. Create it while mapping with the "
                "slam_toolbox/serialize_map service, or pass "
                "posegraph:=<basename-of-a-serialized-map>.{1}".format(
                    path, hint))
        return path


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')

    odom_ekf_config_default = os.path.join(pkg_share, 'config', 'fusion.yaml')
    amcl_config_default = os.path.join(pkg_share, 'config', 'amcl.yaml')
    slam_localization_config_default = os.path.join(
        pkg_share, 'config', 'f1tenth_localization.yaml')

    # --- Shared launch args ---
    odom_ekf_config_la = DeclareLaunchArgument(
        'odom_ekf_config',
        default_value=odom_ekf_config_default,
        description='Config for the ODOM EKF (vesc + imu, world_frame=odom)'
    )

    # `localization_mode` selects between the flows described above. `amcl`
    # and `slam_toolbox` are both "known-map localization instead of
    # rtis_lidar_odom"; the lidar odom node is not started by this launch file
    # in any mode -- in lidar_odom mode you start it separately in the drive
    # container.
    localization_mode_la = DeclareLaunchArgument(
        'localization_mode',
        default_value='lidar_odom',
        description='lidar_odom (default, drift-bounded odom, no map) | amcl '
                    '(known-map particle-filter localization; requires '
                    'map:=<path>.yaml) | slam_toolbox (known-map pose-graph '
                    'localization; requires a serialized pose graph)'
    )
    mode = LaunchConfiguration('localization_mode')
    is_amcl = PythonExpression(['"', mode, '" == "amcl"'])
    is_slam_toolbox = PythonExpression(['"', mode, '" == "slam_toolbox"'])
    # Unknown modes fall back to the historical lidar_odom behavior.
    is_lidar_odom = NotSubstitution(OrSubstitution(is_amcl, is_slam_toolbox))
    # Both map-based localizers need the EKF to own odom -> base_link.
    ekf_owns_odom_tf = OrSubstitution(is_amcl, is_slam_toolbox)

    # --- AMCL-specific args (only used when localization_mode:=amcl) ---
    map_la = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Absolute path to the map .yaml for AMCL mode (required '
                    'when localization_mode:=amcl). Also accepted as the '
                    'pose-graph base for slam_toolbox mode (the extension is '
                    'stripped).'
    )
    amcl_config_la = DeclareLaunchArgument(
        'amcl_config',
        default_value=amcl_config_default,
        description='Config yaml for nav2_amcl + map_server (amcl.yaml)'
    )
    use_map_server_la = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Launch nav2_map_server in amcl mode. Set to false to '
                    'reuse an externally-provided /map.'
    )

    # --- slam_toolbox localization args (only used in that mode) ---
    slam_localization_config_la = DeclareLaunchArgument(
        'slam_localization_config',
        default_value=slam_localization_config_default,
        description='Config yaml for slam_toolbox localization mode '
                    '(f1tenth_localization.yaml)'
    )
    posegraph_la = DeclareLaunchArgument(
        'posegraph',
        default_value='',
        description='Path to the serialized slam_toolbox pose graph for '
                    'localization_mode:=slam_toolbox, without the '
                    '.posegraph/.data suffix. Defaults to the map arg with its '
                    'extension stripped.'
    )
    map_start_pose_la = DeclareLaunchArgument(
        'map_start_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='Robot [x, y, theta] in the map frame at startup for '
                    'localization_mode:=slam_toolbox. The default is the map '
                    'origin, where mapping usually started; pass the actual '
                    'pose if the car starts elsewhere, or set /initialpose.'
    )

    posegraph_basename = PoseGraphBasename(
        LaunchConfiguration('posegraph'), LaunchConfiguration('map'))

    ld = LaunchDescription([
        odom_ekf_config_la,
        localization_mode_la,
        map_la,
        amcl_config_la,
        use_map_server_la,
        slam_localization_config_la,
        posegraph_la,
        map_start_pose_la,
        # Sanity log so users see which mode is active.
        LogInfo(msg=['Localization mode: ', mode]),
        # Resolve (and validate) the pose-graph path before any node starts, so
        # a bad or missing path fails the launch cleanly instead of leaving
        # the EKF and static TFs running.
        LogInfo(
            msg=['slam_toolbox pose graph: ', posegraph_basename],
            condition=IfCondition(is_slam_toolbox),
        ),
    ])

    # --- ODOM EKF (vesc + imu) -- always running ---
    # In lidar_odom mode: publish_tf stays false (per fusion.yaml). The lidar
    # odom node owns odom -> base_link. In the map-localization modes: override
    # publish_tf to true so the EKF owns odom -> base_link itself and the
    # localizer has a motion model to propagate between scans.
    odom_ekf_node_lidar_odom_mode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('odom_ekf_config')],
        condition=IfCondition(is_lidar_odom),
    )
    odom_ekf_node_map_mode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('odom_ekf_config'),
            {'publish_tf': True},
        ],
        condition=IfCondition(ekf_owns_odom_tf),
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
                '"', mode, '" == "amcl"',
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

    # --- slam_toolbox in localization mode ---
    # The executable (not a `mode` parameter) selects pose-graph localization.
    # localization_slam_toolbox_node self-configures, deserializes the pose
    # graph named by map_file_name, and then publishes map -> odom plus a
    # latched /map. It accepts the same /initialpose topic as AMCL for
    # relocalization.
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_localization_config'),
            {
                'use_sim_time': False,
                'map_file_name': posegraph_basename,
                'map_start_pose': ParameterValue(
                    LaunchConfiguration('map_start_pose'),
                    value_type=List[float],
                ),
            },
        ],
        condition=IfCondition(is_slam_toolbox),
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
    ld.add_action(odom_ekf_node_map_mode)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(slam_localization_node)
    ld.add_action(vesc_static_tf_node)
    ld.add_action(vesc_imu_static_tf_node)

    return ld
