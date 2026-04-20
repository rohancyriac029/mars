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
