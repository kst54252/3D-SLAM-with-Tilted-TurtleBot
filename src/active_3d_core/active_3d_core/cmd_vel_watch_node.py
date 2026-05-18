# Copyright 2026 wanjunkim

"""Log cmd_vel commands at a throttled rate for navigation debugging."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelWatchNode(Node):
    """Subscribe to a Twist topic and report representative commands."""

    def __init__(self):
        super().__init__('cmd_vel_watch_node')

        self.declare_parameter('cmd_vel_topic', '/model/tilted_turtlebot/cmd_vel')
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('deadband', 0.005)

        self._cmd_vel_topic = (
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        )
        self._log_period_sec = max(
            0.1,
            self.get_parameter('log_period_sec').get_parameter_value().double_value,
        )
        self._deadband = max(
            0.0,
            self.get_parameter('deadband').get_parameter_value().double_value,
        )
        self._last_log_time = self.get_clock().now()
        self._last_message_time = None

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_callback, 10)
        self.create_timer(2.0, self._timer_callback)

        self.get_logger().info('Watching cmd_vel topic %s' % self._cmd_vel_topic)

    def _cmd_callback(self, msg):
        self._last_message_time = self.get_clock().now()
        now = self._last_message_time
        if (now - self._last_log_time).nanoseconds < self._log_period_sec * 1e9:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        if abs(linear_x) < self._deadband and abs(angular_z) < self._deadband:
            return

        self._last_log_time = now
        self.get_logger().info(
            'cmd_vel linear.x=%.3f angular.z=%.3f' % (linear_x, angular_z)
        )

    def _timer_callback(self):
        if self._last_message_time is None:
            self.get_logger().warn('No cmd_vel messages received yet')
            return

        age = (self.get_clock().now() - self._last_message_time).nanoseconds / 1e9
        if age > 2.0:
            self.get_logger().warn('No cmd_vel messages for %.1f s' % age)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchNode()

    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
