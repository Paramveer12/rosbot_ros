"""ROSbot 3 in Husarion world with Nav2 and AMCL, for RoboCI.

Starts headless Gazebo Harmonic on the provided world file, sets up the clock
bridge, spawns the ROSbot 3 with RViz and arm disabled, and starts localization
and Nav2 on a timer after the controller manager and EKF are up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument('world', description='Path to the world .sdf file'),
    DeclareLaunchArgument('map', default_value='',
                          description='Path to the map YAML file (empty: husarion_world default)'),
    DeclareLaunchArgument('scan_topic', default_value='',
                          description="Nav2 costmap scan topic (empty: /scan)"),
    DeclareLaunchArgument('amcl_scan_topic', default_value='',
                          description="AMCL scan topic (empty: /scan)"),
    DeclareLaunchArgument('nav2_delay', default_value='20.0',
                          description='Seconds after spawn before localization starts (Nav2 starts 10s later)'),
    DeclareLaunchArgument('x', default_value='0.0', description='Initial robot x position'),
    DeclareLaunchArgument('y', default_value='0.0', description='Initial robot y position'),
    DeclareLaunchArgument('z', default_value='0.0', description='Initial robot z position'),
    DeclareLaunchArgument('roll', default_value='0.0', description='Initial robot roll orientation'),
    DeclareLaunchArgument('pitch', default_value='0.0', description='Initial robot pitch orientation'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw orientation'),
]


def launch_setup(context, *args, **kwargs):
    world_file = LaunchConfiguration('world').perform(context)
    if world_file.startswith('package://'):
        pkg, rel = world_file[len('package://'):].split('/', 1)
        world_file = os.path.join(get_package_share_directory(pkg), rel)

    map_arg = LaunchConfiguration('map').perform(context)
    if map_arg.startswith('package://'):
        pkg, rel = map_arg[len('package://'):].split('/', 1)
        map_file = os.path.join(get_package_share_directory(pkg), rel)
    elif map_arg:
        map_file = map_arg
    else:
        map_file = os.path.join(
            get_package_share_directory('husarion_gz_worlds'), 'maps', 'husarion_world.yaml')

    pkg_husarion_gz_worlds = get_package_share_directory('husarion_gz_worlds')
    pkg_rosbot_gazebo = get_package_share_directory('rosbot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    share_paths = [os.path.join(p, 'share') for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]
    resource_path = ':'.join([
        os.path.join(pkg_husarion_gz_worlds, 'models'),
        os.path.join(pkg_husarion_gz_worlds, 'worlds'),
        *share_paths,
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', f'{world_file} -r -s --headless-rendering -v 2')],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    spawn = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_rosbot_gazebo, 'launch', 'spawn_robot.yaml')),
        launch_arguments=[
            ('robot_model', 'rosbot'),
            ('arm_activate', 'False'),
            ('rviz', 'False'),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('roll', LaunchConfiguration('roll')),
            ('pitch', LaunchConfiguration('pitch')),
            ('yaw', LaunchConfiguration('yaw')),
        ],
    )

    roboci_dir = os.path.dirname(__file__)

    eval_amcl_scan_topic = LaunchConfiguration('amcl_scan_topic').perform(context)
    if not eval_amcl_scan_topic:
        eval_amcl_scan_topic = '/scan'

    rewritten_localization_params = RewrittenYaml(
        source_file=os.path.join(roboci_dir, 'localization.yaml'),
        param_rewrites={'scan_topic': eval_amcl_scan_topic},
        convert_types=True,
    )

    eval_scan_topic = LaunchConfiguration('scan_topic').perform(context)
    if not eval_scan_topic:
        eval_scan_topic = '/scan'

    rewritten_nav2_params = RewrittenYaml(
        source_file=os.path.join(roboci_dir, 'nav2.yaml'),
        param_rewrites={'topic': eval_scan_topic},
        convert_types=True,
    )

    localization = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments=[
                ('map', map_file),
                ('use_sim_time', 'true'),
                ('params_file', rewritten_localization_params),
                ('autostart', 'true'),
            ],
        ),
    ])

    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('params_file', rewritten_nav2_params),
                ('autostart', 'true'),
            ],
        ),
    ])

    delay = float(LaunchConfiguration('nav2_delay').perform(context))
    loc_timer = TimerAction(
        period=delay,
        actions=[localization],
    )
    nav2_timer = TimerAction(
        period=delay + 10.0,
        actions=[nav2],
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        clock_bridge,
        spawn,
        loc_timer,
        nav2_timer,
    ]


def generate_launch_description():
    return LaunchDescription([*ARGUMENTS, OpaqueFunction(function=launch_setup)])

