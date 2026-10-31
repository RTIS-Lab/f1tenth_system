"""Nav2 navigation stack for the F1TENTH car.

Starts the navigation servers only. Localization is owned by
fusion.launch.py (localization_mode:=amcl or localization_mode:=slam_toolbox),
and sensors and drivers by bringup_launch.py:

    controller_server    RegulatedPurePursuit (Ackermann path tracking)
    smoother_server      (standby; smoothing happens inside SmacPlannerHybrid)
    planner_server       SmacPlannerHybrid, DUBIN motion model
    behavior_server      wait-only recovery (the car cannot spin)
    bt_navigator         /navigate_to_pose action + /goal_pose subscription
    waypoint_follower    /follow_waypoints (unused for clicked-point goals)
    velocity_smoother    /cmd_vel_nav -> /cmd_vel (acceleration-limited Twist)
    lifecycle_manager_navigation

Sending goals: RViz's "2D Goal Pose" tool publishes a PoseStamped on
/goal_pose and bt_navigator turns each message into a NavigateToPose goal.
A new click preempts the active goal, so clicking around the map is a
natural way to drive. The Nav2 RViz panel works too.

Drive path: /cmd_vel (this stack) -> teleop twist bridge (drive container,
only while the RUN_AUTO joystick button is held) -> /teleop/drive ->
safety stack -> vesc. Nothing moves unless RUN_AUTO is held.

Usage (system container):
    ros2 launch f1tenth_stack navigation.launch.py
Override params or disable autostart:
    ros2 launch f1tenth_stack navigation.launch.py \
        params_file:=/abs/path/to/nav2_params.yaml autostart:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_bt_xml = os.path.join(
        pkg_share, 'launch', 'navigate_to_pose_ackermann.xml')
    through_poses_bt_xml = os.path.join(
        pkg_share, 'launch', 'navigate_through_poses_ackermann.xml')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Map fully qualified names to relative ones so the node's namespace can
    # be prepended (mirrors nav2_bringup/navigation_launch.py).
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    params_file_la = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for all nav nodes')
    autostart_la = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate the nav2 lifecycle nodes')

    ld = LaunchDescription([params_file_la, autostart_la])
    params_file = LaunchConfiguration('params_file')

    # The controller's output is remapped to cmd_vel_nav (the velocity
    # smoother's input); the smoother publishes the final /cmd_vel that the
    # teleop bridge in the drive container consumes.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings)
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            # bt_navigator loads BOTH default BTs at configure, so both must
            # be spin-free: the stock through-poses BT contains Spin and
            # would fail to load against the wait-only behavior_server.
            {'default_nav_to_pose_bt_xml': default_bt_xml},
            {'default_nav_through_poses_bt_xml': through_poses_bt_xml},
        ],
        remappings=remappings)
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=remappings)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'node_names': lifecycle_nodes,
            'autostart': LaunchConfiguration('autostart'),
            'bond_timeout': 4.0,
        }])

    for node in (controller_server, smoother_server, planner_server,
                 behavior_server, bt_navigator, waypoint_follower,
                 velocity_smoother, lifecycle_manager):
        ld.add_action(node)

    return ld
