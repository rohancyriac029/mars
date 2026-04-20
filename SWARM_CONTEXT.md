# Goal-Based Multi-Robot Swarm Coordination (ROS 2 Humble) - Full Context

## 1) Objective
Build a minimal and robust multi-robot system in Gazebo + Nav2 where:
- 4 robots run in separate namespaces
- robot1 is leader
- robot2, robot3, robot4 are followers
- followers receive offset goals based on leader goal/pose

---

## 2) Methodology

### Architecture
- Simulator: Gazebo Classic
- Navigation: Nav2 (per robot namespace)
- Visualization: RViz2 (single instance from robot1 namespace)
- Custom coordination: Python node (`goal_coordinator.py`)

### Design choices
1. Keep standard Nav2 stack, no SLAM customization, no custom planner.
2. Use namespaced robots (`/robot1` ... `/robot4`) to avoid topic/TF collisions.
3. Publish a leader goal and deterministic follower offsets.
4. Optionally replan follower goals using leader odometry.
5. Delay coordinator start until Nav2 action servers are likely available.
6. Republish initial poses multiple times to improve AMCL startup stability.

---

## 3) Workspace Structure

```text
mars/
  src/
    swarm_coordinator/
      launch/
        multi_robot_swarm.launch.py
      swarm_coordinator/
        __init__.py
        goal_coordinator.py
      resource/
        swarm_coordinator
      package.xml
      setup.py
      setup.cfg
```

---

## 4) Full Current Code

### `src/swarm_coordinator/package.xml`
```xml
<?xml version="1.0"?>
<package format="3">
  <name>swarm_coordinator</name>
  <version>0.0.1</version>
  <description>Minimal multi-robot swarm goal coordinator using Nav2 actions.</description>
  <maintainer email="you@example.com">swarm_user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>turtlebot3_gazebo</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `src/swarm_coordinator/setup.py`
```python
from setuptools import setup

package_name = 'swarm_coordinator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_swarm.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm_user',
    maintainer_email='you@example.com',
    description='Minimal multi-robot swarm goal coordinator using Nav2 actions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_coordinator = swarm_coordinator.goal_coordinator:main',
        ],
    },
)
```

### `src/swarm_coordinator/setup.cfg`
```ini
[develop]
script_dir=$base/lib/swarm_coordinator

[install]
install_scripts=$base/lib/swarm_coordinator
```

### `src/swarm_coordinator/resource/swarm_coordinator`
```text
swarm_coordinator
```

### `src/swarm_coordinator/swarm_coordinator/__init__.py`
```python

```

### `src/swarm_coordinator/launch/multi_robot_swarm.launch.py`
```python
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
        {'name': 'robot2', 'x': -1.5, 'y': 0.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot3', 'x': 1.5, 'y': 0.0, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot4', 'x': 0.0, 'y': -1.5, 'z': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
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
        start_delay = float(idx * 8)

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
        period=90.0,
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
                        'leader_goal': [1.0, 0.0, 0.0],
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
```

### `src/swarm_coordinator/swarm_coordinator/goal_coordinator.py`
```python
#!/usr/bin/env python3
import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalCoordinator(Node):
    """Leader-follower goal coordinator for namespaced Nav2 robots."""

    def __init__(self) -> None:
        super().__init__('goal_coordinator')

        self.leader_ns = str(self.declare_parameter('leader_ns', 'robot1').value)
        self.follower_ns = list(
            self.declare_parameter('follower_ns', ['robot2', 'robot3', 'robot4']).value
        )
        self.leader_goal = list(self.declare_parameter('leader_goal', [1.0, 0.0, 0.0]).value)
        raw_dynamic_follow = self.declare_parameter('dynamic_follow', False).value
        if isinstance(raw_dynamic_follow, str):
            self.dynamic_follow = raw_dynamic_follow.lower() in ('1', 'true', 'yes', 'on')
        else:
            self.dynamic_follow = bool(raw_dynamic_follow)
        self.replan_period = float(self.declare_parameter('replan_period', 2.0).value)
        self.goal_update_threshold = float(
            self.declare_parameter('goal_update_threshold', 0.30).value
        )
        self.send_initial_pose = bool(self.declare_parameter('send_initial_pose', True).value)
        self.initial_pose_republish_count = int(
            self.declare_parameter('initial_pose_republish_count', 30).value
        )

        # Fixed offsets around the leader in map coordinates.
        self.offsets: Dict[str, Tuple[float, float]] = {
            'robot2': (-1.0, 0.5),
            'robot3': (-1.0, -0.5),
            'robot4': (-2.0, 0.0),
        }

        # Must match the spawn poses defined in the launch file.
        self.spawn_poses: Dict[str, Tuple[float, float, float]] = {
            'robot1': (0.0, 0.0, 0.0),
            'robot2': (-1.5, 0.0, 0.0),
            'robot3': (1.5, 0.0, 0.0),
            'robot4': (0.0, -1.5, 0.0),
        }

        self.robot_names = [self.leader_ns] + self.follower_ns
        self.nav_action_clients: Dict[str, ActionClient] = {}
        self.initial_pose_publishers = {}

        for ns in self.robot_names:
            self.nav_action_clients[ns] = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
            self.initial_pose_publishers[ns] = self.create_publisher(
                PoseWithCovarianceStamped, f'/{ns}/initialpose', 10
            )

        self.leader_pose: Optional[Tuple[float, float, float]] = None
        self.last_follower_goals: Dict[str, Tuple[float, float, float]] = {}
        self.leader_odom_sub = None

        if self.dynamic_follow:
            self.leader_odom_sub = self.create_subscription(
                Odometry, f'/{self.leader_ns}/odom', self._leader_odom_cb, 10
            )
            self.dynamic_timer = self.create_timer(self.replan_period, self._dynamic_replan)
        else:
            self.dynamic_timer = None

        self.bootstrap_count = 0
        self.formation_sent = False
        self.bootstrap_timer = self.create_timer(1.0, self._bootstrap_step)

        self.get_logger().info(
            f'Coordinator ready. Leader=/{self.leader_ns}, followers={self.follower_ns}, '
            f'dynamic_follow={self.dynamic_follow}'
        )

    def _bootstrap_step(self) -> None:
        self.bootstrap_count += 1

        # Publish initial pose a few times so AMCL receives it after startup.
        if self.send_initial_pose and self.bootstrap_count <= self.initial_pose_republish_count:
            for ns in self.robot_names:
                if ns in self.spawn_poses:
                    x, y, yaw = self.spawn_poses[ns]
                    self._publish_initial_pose(ns, x, y, yaw)

        if self.formation_sent:
            return

        all_ready = all(
            client.wait_for_server(timeout_sec=0.0)
            for client in self.nav_action_clients.values()
        )
        if not all_ready:
            if self.bootstrap_count % 5 == 0:
                self.get_logger().info('Waiting for Nav2 action servers...')
            return

        self._send_initial_formation()
        self.formation_sent = True
        self.bootstrap_timer.cancel()

    def _send_initial_formation(self) -> None:
        lx, ly, lyaw = self.leader_goal

        self._send_nav_goal(self.leader_ns, lx, ly, lyaw)
        self.get_logger().info(
            f'Leader goal: /{self.leader_ns} -> ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f})'
        )

        for ns in self.follower_ns:
            dx, dy = self.offsets.get(ns, (0.0, 0.0))
            gx = lx + dx
            gy = ly + dy
            self._send_nav_goal(ns, gx, gy, lyaw)
            self.last_follower_goals[ns] = (gx, gy, lyaw)
            self.get_logger().info(f'Follower goal: /{ns} -> ({gx:.2f}, {gy:.2f}, yaw={lyaw:.2f})')

    def _leader_odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.leader_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quat(q),
        )

    def _dynamic_replan(self) -> None:
        if not self.dynamic_follow or not self.formation_sent or self.leader_pose is None:
            return

        lx, ly, lyaw = self.leader_pose

        for ns in self.follower_ns:
            dx, dy = self.offsets.get(ns, (0.0, 0.0))
            gx = lx + dx
            gy = ly + dy
            candidate = (gx, gy, lyaw)

            previous = self.last_follower_goals.get(ns)
            if previous is not None and self._pose_delta(previous, candidate) < self.goal_update_threshold:
                continue

            self._send_nav_goal(ns, gx, gy, lyaw)
            self.last_follower_goals[ns] = candidate

    def _send_nav_goal(self, robot_ns: str, x: float, y: float, yaw: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self._quat_from_yaw(yaw)

        future = self.nav_action_clients[robot_ns].send_goal_async(goal)
        future.add_done_callback(lambda f, ns=robot_ns: self._goal_response_cb(ns, f))

    def _goal_response_cb(self, robot_ns: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send goal to /{robot_ns}: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warning(f'Goal rejected by /{robot_ns}')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, ns=robot_ns: self._goal_result_cb(ns, f))

    def _goal_result_cb(self, robot_ns: str, future) -> None:
        try:
            status = future.result().status
            self.get_logger().info(f'/{robot_ns} goal finished with status={status}')
        except Exception as exc:
            self.get_logger().error(f'Error while getting result from /{robot_ns}: {exc}')

    def _publish_initial_pose(self, robot_ns: str, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._quat_from_yaw(yaw)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07

        self.initial_pose_publishers[robot_ns].publish(msg)

    @staticmethod
    def _quat_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _pose_delta(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None) -> None:
    node = None
    rclpy.init(args=args)
    node = GoalCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5) Commands (Recommended)

### Build
```bash
cd ~/gogo/mars
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select swarm_coordinator
source install/setup.bash
```

### Run custom launch
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch swarm_coordinator multi_robot_swarm.launch.py
```

### Run dynamic follower mode
```bash
ros2 launch swarm_coordinator multi_robot_swarm.launch.py dynamic_follow:=True
```

### Start Nav2 multirobot directly (reference command)
Important: all args must be part of the same command.
```bash
ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py \
robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: -1.5, y: 0.0, yaw: 0.0}; robot3={x: 1.5, y: 0.0, yaw: 0.0}; robot4={x: 0.0, y: -1.5, yaw: 0.0}" \
world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
use_rviz:=False autostart:=True \
params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

---

## 6) Recent Error Context and Root Causes

### Error A: `number_of_robots=0`
Observed in logs:
- `number_of_robots=0`
- `autostart: false`
- default params used instead of passed params

Root cause:
- Launch arguments were entered as separate shell lines, not attached to `ros2 launch` command.

Fix:
- Keep all arguments in one command or use line continuation (`\\`).

---

### Error B: malformed robots pose parsing
Observed in logs:
- `init_pose={'x:0.0': None, ... 'x': 0.0, 'y': 0.0, ...}`

Root cause:
- Pose format used `x:0.0` instead of `x: 0.0`.

Fix:
- Use valid YAML style with space after colon.

---

### Error C: `Could not load library: libnav2_are_error_codes_active_condition_bt_node.so`
Observed while using multirobot params all file.

Root cause:
- Installed Nav2 binary set did not include plugin expected by `nav2_multirobot_params_all.yaml` plugin list.

Fix:
- Switched default params to `nav2_params.yaml`.

---

### Error D: `spawn_entity` unavailable / no odom
Observed:
- `/spawn_entity` timeout or no robot models in Gazebo.
- Repeated Nav2 TF timeout: `base_link -> odom` frame does not exist.

Root cause:
- Robot models not spawned successfully, so odom publishers are absent.

Fixes applied:
- Added `GAZEBO_MODEL_PATH` setup in custom launch.
- Added staggered robot startup.
- Ensured one robot starts simulator and others join with `use_simulator=False`.

---

### Error E: `rcl_shutdown already called`
Observed in `goal_coordinator` on teardown.

Root cause:
- Double shutdown path during interrupted exits.

Fix:
- Added `if rclpy.ok(): rclpy.shutdown()` guard and safe node destroy block.

---

### Error F: invalid topic echo usage
Observed:
- `ros2 topic echo /gazebo/model_states -`
- `The passed message type is invalid`

Root cause:
- Trailing `-` interpreted as message type argument.

Fix:
- Use `ros2 topic echo /model_states --once` or `ros2 topic echo /gazebo/model_states --once`.

---

## 7) Quick Runtime Validation Checklist

1. Verify robot count in launch log:
- `number_of_robots=4`

2. Verify spawn service:
```bash
ros2 service list | grep spawn_entity
```

3. Verify model states topic:
```bash
ros2 topic list | grep model_states
ros2 topic echo /model_states --once
```

4. Verify odom exists:
```bash
ros2 topic echo /robot1/odom --once
```

5. Verify action servers:
```bash
ros2 action list | grep navigate_to_pose
```

---

## 8) Notes
- The map shown in RViz is loaded from Nav2 map server:
  - `/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml`
- Gazebo world is separate:
  - `/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world`
- For clean shutdown, press Ctrl+C once and wait.

---

## 9) Full UI Addendum (Latest)

These updates were applied to improve full UI behavior on Ubuntu desktop (Gazebo GUI + one RViz):

1. Added `TURTLEBOT3_MODEL` env setup inside launch using a `tb3_model` launch arg.
2. Added RViz config fallback:
     - prefer `nav2_namespaced_view.rviz`
     - fallback to `nav2_default_view.rviz` if missing
3. Increased coordinator delayed start from `35.0` to `90.0` seconds for slower machines.
4. Kept one RViz instance and staggered robot startup.

### Critical CLI pitfall (causes `number_of_robots=0`)

If you split launch arguments onto separate lines *without* `\\`, they are ignored by `ros2 launch` and defaults are used.

Wrong:
```bash
ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py
robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; ..."
```

Correct:
```bash
ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py \
    robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: -1.5, y: 0.0, yaw: 0.0}; robot3={x: 1.5, y: 0.0, yaw: 0.0}; robot4={x: 0.0, y: -1.5, yaw: 0.0}" \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    use_rviz:=False autostart:=True \
    params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### Topic echo pitfall

Do not use trailing `-`:

Wrong:
```bash
ros2 topic echo /gazebo/model_states -
```

Correct:
```bash
ros2 topic echo /model_states --once
ros2 topic echo /gazebo/model_states --once
```

### Source of truth note

If this markdown and code ever differ, use workspace code as source of truth:
- `src/swarm_coordinator/launch/multi_robot_swarm.launch.py`
- `src/swarm_coordinator/swarm_coordinator/goal_coordinator.py`

---

## 10) Gazebo Transport Crash (Stale Master) - Latest Root Cause

### Symptom pattern

Observed in logs:
- Gazebo exits early with transport/master errors (for example exit code `255`, unable to initialize transport, unable to read from master).
- Then no robots are spawned.
- Then Nav2 loops on TF timeout:
    - `Timed out waiting for transform from base_link to odom`.

### Why this causes all downstream failures

If Gazebo dies before robot spawn:
1. No `/spawn_entity` success
2. No robot odometry topics (`/robotX/odom`)
3. No `odom` TF for each robot
4. Local/global costmaps cannot transform frames
5. Navigation appears frozen and goals are rejected or never executed

### Confirm stale master/port conflict

```bash
ss -tlnp | grep 11345
```

If anything is still bound to `11345` after shutdown, Gazebo relaunch can fail.

### Full cleanup before relaunch

```bash
# Kill lingering simulator + ROS launch processes
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f gazebo || true
pkill -9 -f "ros2 launch" || true
pkill -9 -f "ros2 run" || true

# Give OS time to release resources
sleep 5

# Verify Gazebo master port is free (expect no output)
ss -tlnp | grep 11345

# Clear Gazebo shared-memory artifacts (best effort)
rm -f /tmp/gzmaster* /tmp/.gazebo* 2>/dev/null || true
```

### Relaunch (single command with continuations)

```bash
source /opt/ros/humble/setup.bash
source ~/gogo/mars/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py \
    robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: -1.5, y: 0.0, yaw: 0.0}; robot3={x: 1.5, y: 0.0, yaw: 0.0}; robot4={x: 0.0, y: -1.5, yaw: 0.0}" \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    use_rviz:=False \
    autostart:=True \
    params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### Healthy startup indicators

Look for lines like:
- `Connected to gazebo master @ http://127.0.0.1:11345`
- `Publicized address: ...`

Then verify robots actually exist:

```bash
ros2 topic list | grep -E "model_states|robot[1-4]/odom"
ros2 topic echo /model_states --once
ros2 topic echo /robot1/odom --once
```

Only start coordinator after odom/action servers are visible.

### Wayland/X11 compatibility note

If Gazebo GUI still fails under Wayland sessions, force X11 backend:

```bash
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0   # or :1 depending on active session
```
