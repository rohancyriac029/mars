#!/usr/bin/env python3
import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalCoordinator(Node):
    """Leader-follower goal coordinator for namespaced Nav2 robots."""

    def __init__(self) -> None:
        super().__init__('goal_coordinator')

        self.leader_ns = str(self.declare_parameter('leader_ns', 'robot1').value)
        self.follower_ns = list(
            self.declare_parameter('follower_ns', ['robot2', 'robot3']).value
        )
        self.leader_goal = list(self.declare_parameter('leader_goal', [1.5, 0.0, 0.0]).value)
        raw_dynamic_follow = self.declare_parameter('dynamic_follow', False).value
        if isinstance(raw_dynamic_follow, str):
            self.dynamic_follow = raw_dynamic_follow.lower() in ('1', 'true', 'yes', 'on')
        else:
            self.dynamic_follow = bool(raw_dynamic_follow)
        self.replan_period = float(self.declare_parameter('replan_period', 2.0).value)
        self.goal_update_threshold = float(
            self.declare_parameter('goal_update_threshold', 0.30).value
        )
        self.leader_goal_topic = str(
            self.declare_parameter('leader_goal_topic', f'/{self.leader_ns}/goal_pose').value
        )
        self.leader_plan_topic = str(
            self.declare_parameter('leader_plan_topic', f'/{self.leader_ns}/plan').value
        )
        self.goal_relay_position_tolerance = float(
            self.declare_parameter('goal_relay_position_tolerance', 0.15).value
        )
        self.follower_trailing_offset = float(
            self.declare_parameter('follower_trailing_offset', -0.30).value
        )
        self.follower_lateral_spacing = float(
            self.declare_parameter('follower_lateral_spacing', 0.60).value
        )
        self.map_min_x = float(self.declare_parameter('map_min_x', -2.2).value)
        self.map_max_x = float(self.declare_parameter('map_max_x', 2.2).value)
        self.map_min_y = float(self.declare_parameter('map_min_y', -2.2).value)
        self.map_max_y = float(self.declare_parameter('map_max_y', 2.2).value)
        self.follower_goal_margin = float(
            self.declare_parameter('follower_goal_margin', 0.25).value
        )
        self.send_initial_pose = bool(self.declare_parameter('send_initial_pose', True).value)
        self.initial_pose_republish_count = int(
            self.declare_parameter('initial_pose_republish_count', 30).value
        )

        # Fixed offsets around the leader in map coordinates.
        self.offsets: Dict[str, Tuple[float, float]] = {
            'robot2': (self.follower_trailing_offset, -self.follower_lateral_spacing),
            'robot3': (self.follower_trailing_offset, self.follower_lateral_spacing),
        }

        # Must match the spawn poses defined in the launch file.
        self.spawn_poses: Dict[str, Tuple[float, float, float]] = {
            'robot1': (-2.0, 0.0, 0.0),
            'robot2': (-2.0, -1.0, 0.0),
            'robot3': (-2.0, 1.0, 0.0),
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

        # Listen to Nav2 goal tool clicks and broadcast the formation goal.
        self.leader_goal_sub = self.create_subscription(
            PoseStamped, self.leader_goal_topic, self._leader_goal_cb, 10
        )
        self.leader_plan_sub = self.create_subscription(
            Path, self.leader_plan_topic, self._leader_plan_cb, 10
        )
        self.global_goal_sub = None
        if self.leader_goal_topic != '/goal_pose':
            self.global_goal_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self._leader_goal_cb, 10
            )

        self.bootstrap_count = 0
        self.formation_sent = False
        self.skip_bootstrap_leader_send = False
        self.bootstrap_timer = self.create_timer(1.0, self._bootstrap_step)

        self.get_logger().info(
            f'Coordinator ready. Leader=/{self.leader_ns}, followers={self.follower_ns}, '
            f'dynamic_follow={self.dynamic_follow}'
        )
        self.get_logger().info(
            f'Listening for leader goals on {self.leader_goal_topic}'
        )
        self.get_logger().info(
            f'Listening for leader plans on {self.leader_plan_topic}'
        )
        if self.global_goal_sub is not None:
            self.get_logger().info('Also listening for leader goals on /goal_pose')

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

        include_leader = not self.skip_bootstrap_leader_send
        self._send_formation_goal(lx, ly, lyaw, include_leader=include_leader)

    def _leader_goal_cb(self, msg: PoseStamped) -> None:
        lx = float(msg.pose.position.x)
        ly = float(msg.pose.position.y)
        lyaw = self._yaw_from_quat(msg.pose.orientation)
        self._relay_formation_if_new_goal(
            lx, ly, lyaw, source='goal_topic', include_leader=True
        )

    def _leader_plan_cb(self, msg: Path) -> None:
        # Nav2 Goal in RViz uses actions directly; infer new leader goals from plan endpoint.
        if not msg.poses:
            return

        goal_pose = msg.poses[-1].pose
        lx = float(goal_pose.position.x)
        ly = float(goal_pose.position.y)
        lyaw = self._yaw_from_quat(goal_pose.orientation)
        self._relay_formation_if_new_goal(
            lx, ly, lyaw, source='leader_plan', include_leader=False
        )

    def _relay_formation_if_new_goal(
        self,
        lx: float,
        ly: float,
        lyaw: float,
        source: str,
        include_leader: bool,
    ) -> None:
        previous = (self.leader_goal[0], self.leader_goal[1], self.leader_goal[2])
        candidate = (lx, ly, lyaw)
        if self._pose_delta(previous, candidate) < self.goal_relay_position_tolerance:
            return

        self.leader_goal = [lx, ly, lyaw]

        if not self.formation_sent:
            if source == 'leader_plan' and not include_leader:
                # RViz Nav2 goal already commands robot1 directly; don't re-preempt on bootstrap.
                self.skip_bootstrap_leader_send = True
            self.get_logger().info(
                f'Queued leader goal from {source} before formation ready '
                f'-> ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f})'
            )
            return

        self.get_logger().info(
            f'New leader goal from {source} -> ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f})'
        )
        self._send_formation_goal(lx, ly, lyaw, include_leader=include_leader)

    def _send_formation_goal(
        self, lx: float, ly: float, lyaw: float, include_leader: bool
    ) -> None:

        if include_leader:
            self._send_nav_goal(self.leader_ns, lx, ly, lyaw)
            self.get_logger().info(
                f'Leader goal: /{self.leader_ns} -> ({lx:.2f}, {ly:.2f}, yaw={lyaw:.2f})'
            )

        for ns in self.follower_ns:
            dx, dy = self.offsets.get(ns, (0.0, 0.0))
            gx, gy = self._bounded_follower_goal(lx + dx, ly + dy)
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
            gx, gy = self._bounded_follower_goal(lx + dx, ly + dy)
            candidate = (gx, gy, lyaw)

            previous = self.last_follower_goals.get(ns)
            if previous is not None and self._pose_delta(previous, candidate) < self.goal_update_threshold:
                continue

            self._send_nav_goal(ns, gx, gy, lyaw)
            self.last_follower_goals[ns] = candidate

    def _bounded_follower_goal(self, x: float, y: float) -> Tuple[float, float]:
        min_x = self.map_min_x + self.follower_goal_margin
        max_x = self.map_max_x - self.follower_goal_margin
        min_y = self.map_min_y + self.follower_goal_margin
        max_y = self.map_max_y - self.follower_goal_margin

        bx = min(max(x, min_x), max_x)
        by = min(max(y, min_y), max_y)

        if abs(bx - x) > 1e-6 or abs(by - y) > 1e-6:
            self.get_logger().warning(
                f'Follower goal adjusted to map bounds: ({x:.2f}, {y:.2f}) -> ({bx:.2f}, {by:.2f})'
            )
        return bx, by

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
