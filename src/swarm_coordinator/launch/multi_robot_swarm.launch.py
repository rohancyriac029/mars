import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    nav2_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    launch_dir = os.path.join(nav2_dir, 'launch')
    default_world = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    default_map = os.path.join(nav2_dir, 'maps', 'turtlebot3_world.yaml')
    # nav2_params.yaml avoids BT plugins that may be missing in some Humble installs.
    default_params = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
    namespaced_rviz = os.path.join(nav2_dir, 'rviz', 'nav2_namespaced_view.rviz')
    default_rviz = (
        namespaced_rviz
        if os.path.exists(namespaced_rviz)
        else os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')
    )

    gazebo_model_path = os.path.join(tb3_gazebo_dir, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        gazebo_model_path = f'{gazebo_model_path}:{existing_model_path}'

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', gazebo_model_path
    )
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('tb3_model'))

    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot2', 'x': 0.0, 'y': -1.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot3', 'x': 0.0, 'y': 1.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot4', 'x': -1.0, 'y': 0.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    ]

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='Gazebo world file'
    )
    declare_tb3_model = DeclareLaunchArgument(
        'tb3_model',
        default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
        description='TurtleBot3 model used by simulation assets',
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
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Start a single RViz instance'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='True', description='Autostart Nav2 lifecycle nodes'
    )
    declare_dynamic_follow = DeclareLaunchArgument(
        'dynamic_follow',
        default_value='False',
        description='If true, followers keep updating from leader odometry',
    )

    robot_actions = [LogInfo(msg=['number_of_robots=', str(len(robots))])]
    for idx, robot in enumerate(robots):
        use_simulator = 'True' if idx == 0 else 'False'
        headless = 'False' if idx == 0 else 'True'
        start_delay = float(idx * 3)

        robot_actions.append(
            TimerAction(
                period=start_delay,
                actions=[
                    GroupAction(
                        [
                            LogInfo(
                                msg=[
                                    'Launching namespace=',
                                    robot['name'],
                                    ' init_pose=',
                                    str(robot),
                                    ' use_simulator=',
                                    use_simulator,
                                ]
                            ),
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(launch_dir, 'tb3_simulation_launch.py')
                                ),
                                launch_arguments={
                                    'namespace': TextSubstitution(text=robot['name']),
                                    'use_namespace': 'True',
                                    'map': LaunchConfiguration('map'),
                                    'use_sim_time': 'True',
                                    'params_file': LaunchConfiguration('params_file'),
                                    'autostart': LaunchConfiguration('autostart'),
                                    'use_rviz': 'False',
                                    'use_simulator': use_simulator,
                                    'headless': headless,
                                    'world': LaunchConfiguration('world'),
                                    'use_robot_state_pub': 'True',
                                    'x_pose': TextSubstitution(text=str(robot['x'])),
                                    'y_pose': TextSubstitution(text=str(robot['y'])),
                                    'z_pose': TextSubstitution(text=str(robot['z'])),
                                    'roll': TextSubstitution(text=str(robot['roll'])),
                                    'pitch': TextSubstitution(text=str(robot['pitch'])),
                                    'yaw': TextSubstitution(text=str(robot['yaw'])),
                                    'robot_name': TextSubstitution(text=robot['name']),
                                }.items(),
                            ),
                        ]
                    )
                ],
            )
        )

    single_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': TextSubstitution(text='robot1'),
            'use_namespace': 'True',
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    delayed_robot_bringup = TimerAction(period=2.0, actions=robot_actions)

    coordinator = TimerAction(
        period=55.0,
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
            set_gazebo_model_path,
            set_tb3_model,
            declare_world,
            declare_tb3_model,
            declare_map,
            declare_params_file,
            declare_rviz_config,
            declare_use_rviz,
            declare_autostart,
            declare_dynamic_follow,
            delayed_robot_bringup,
            single_rviz,
            coordinator,
        ]
    )
