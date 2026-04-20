#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


class PoseInitializer(Node):
    def __init__(self) -> None:
        super().__init__('pose_initializer')

        self.spawn_poses = {
            'robot1': (-2.0, 0.0, 0.0),
            'robot2': (-2.0, -1.0, 0.0),
            'robot3': (-2.0, 1.0, 0.0),
        }

        self.publishers = {}
        for ns in self.spawn_poses:
            self.publishers[ns] = self.create_publisher(
                PoseWithCovarianceStamped, f'/{ns}/initialpose', 10
            )

        self.count = 0
        self.timer = self.create_timer(1.0, self._publish)
        self.get_logger().info('PoseInitializer started: publishing initial poses...')

    def _publish(self) -> None:
        self.count += 1
        for ns, (x, y, yaw) in self.spawn_poses.items():
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation = self._quat(yaw)

            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.07
            self.publishers[ns].publish(msg)

        if self.count % 5 == 0:
            self.get_logger().info(f'Published initial poses x{self.count}')

        if self.count >= 40:
            self.get_logger().info('Done publishing initial poses.')
            self.timer.cancel()

    @staticmethod
    def _quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
