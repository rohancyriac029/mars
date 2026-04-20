import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    cloned_multi_launch = os.path.join(nav2_dir, 'launch', 'cloned_multi_tb3_simulation_launch.py')
    default_world = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    default_map = os.path.join(nav2_dir, 'maps', 'turtlebot3_world.yaml')
    default_params = os.path.join(nav2_dir, 'params', 'nav2_multirobot_params_all.yaml')
    default_rviz = os.path.join(nav2_dir, 'rviz', 'nav2_namespaced_view.rviz')

    default_robots = (
        'robot1={x: 0.0, y: 0.0, yaw: 0.0}; '
        'robot2={x: 0.0, y: -1.0, yaw: 0.0}; '
        'robot3={x: 0.0, y: 1.0, yaw: 0.0}; '
        'robot4={x: -1.0, y: 0.0, yaw: 0.0}'
    )

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='Gazebo world file'
    )
    declare_map = DeclareLaunchArgument('map', default_value=default_map, description='Nav2 map yaml')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Nav2 shared multirobot params yaml',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz, description='RViz config file'
    )
    declare_robots = DeclareLaunchArgument(
        'robots', default_value=default_robots, description='Robot namespaces and spawn poses'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Start RViz for each robot namespace'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='True', description='Autostart Nav2 lifecycle nodes'
    )
    declare_dynamic_follow = DeclareLaunchArgument(
        'dynamic_follow',
        default_value='False',
        description='If true, followers keep updating from leader odometry',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cloned_multi_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'simulator': 'gazebo',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_robot_state_pub': 'True',
            'robots': LaunchConfiguration('robots'),
            'log_settings': 'true',
        }.items(),
    )

    coordinator = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='swarm_coordinator',
                executable='goal_coordinator',
                name='goal_coordinator',
                output='screen',
                parameters=[
                    {
                        'leader_ns': 'robot1',
                        'follower_ns': ['robot2', 'robot3', 'robot4'],
                        'leader_goal': [2.0, 2.0, 0.0],
                        'dynamic_follow': ParameterValue(
                            LaunchConfiguration('dynamic_follow'), value_type=bool
                        ),
                        'replan_period': 2.0,
                        'send_initial_pose': True,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_world,
            declare_map,
            declare_params_file,
            declare_rviz_config,
            declare_robots,
            declare_use_rviz,
            declare_autostart,
            declare_dynamic_follow,
            simulation,
            coordinator,
        ]
    )
