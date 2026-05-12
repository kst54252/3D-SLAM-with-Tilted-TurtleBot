# Copyright 2026 wanjunkim
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish a timed velocity command, then stop the robot."""

import math
import time

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SpinOnceNode(Node):
    """Drive the robot with a constant velocity, then publish stop commands."""

    def __init__(self):
        """Initialize the command publisher and spin timing."""
        super().__init__('spin_once_node')

        self.declare_parameter('cmd_vel_topic', '/model/tilted_turtlebot/cmd_vel')
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('turn_angle', 6.283185307179586)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('stop_publish_count', 10)
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self._cmd_vel_topic = (
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        )
        self._linear_speed = (
            self.get_parameter('linear_speed').get_parameter_value().double_value
        )
        self._angular_speed = abs(
            self.get_parameter('angular_speed').get_parameter_value().double_value
        )
        self._duration = max(
            0.0,
            self.get_parameter('duration').get_parameter_value().double_value,
        )
        self._turn_angle = abs(
            self.get_parameter('turn_angle').get_parameter_value().double_value
        )
        self._start_delay = max(
            0.0,
            self.get_parameter('start_delay').get_parameter_value().double_value,
        )
        self._stop_publish_count = max(
            1,
            self.get_parameter('stop_publish_count')
            .get_parameter_value()
            .integer_value,
        )
        self._publish_cmd_vel = (
            self.get_parameter('publish_cmd_vel').get_parameter_value().bool_value
        )
        self._publish_tf = (
            self.get_parameter('publish_tf').get_parameter_value().bool_value
        )
        self._parent_frame = (
            self.get_parameter('parent_frame').get_parameter_value().string_value
        )
        self._child_frame = (
            self.get_parameter('child_frame').get_parameter_value().string_value
        )
        self._x = (
            self.get_parameter('initial_x').get_parameter_value().double_value
        )
        self._y = (
            self.get_parameter('initial_y').get_parameter_value().double_value
        )
        self._z = (
            self.get_parameter('initial_z').get_parameter_value().double_value
        )
        self._yaw = (
            self.get_parameter('initial_yaw').get_parameter_value().double_value
        )
        if self._duration > 0.0:
            self._command_duration = self._duration
        else:
            self._command_duration = self._turn_angle / self._angular_speed
        self._start_time = time.monotonic()
        self._last_update_time = self._start_time
        self._stop_count = 0
        self._finished = False

        self._publisher = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.05, self._timer_callback)

        self.get_logger().info(
            'Will publish velocity on %s after %.1f s: %.2f m/s, %.2f rad/s'
            % (
                self._cmd_vel_topic,
                self._start_delay,
                self._linear_speed,
                self._angular_speed,
            )
        )

    def _timer_callback(self):
        """Publish spin or stop commands according to elapsed time."""
        now = time.monotonic()
        elapsed = now - self._start_time
        self._update_open_loop_pose(now, elapsed)
        self._publish_open_loop_tf()

        if self._finished:
            return

        command = Twist()

        if elapsed < self._start_delay:
            self._publish_velocity(command)
            return

        if elapsed < self._start_delay + self._command_duration:
            command.linear.x = self._linear_speed
            command.angular.z = self._angular_speed
            self._publish_velocity(command)
            return

        self._publish_velocity(command)
        self._stop_count += 1
        if self._stop_count >= self._stop_publish_count:
            self.get_logger().info('Completed timed velocity command')
            self._finished = True

    def _publish_velocity(self, command):
        """Publish cmd_vel only when this node owns Gazebo robot motion."""
        if self._publish_cmd_vel:
            self._publisher.publish(command)

    def _update_open_loop_pose(self, now, elapsed):
        """Integrate the commanded velocity for a smooth demo odom frame."""
        previous_elapsed = self._last_update_time - self._start_time
        self._last_update_time = now

        command_start = self._start_delay
        command_end = self._start_delay + self._command_duration
        dt_start = max(previous_elapsed, command_start)
        dt_end = min(elapsed, command_end)
        dt = dt_end - dt_start
        if dt <= 0.0:
            return

        if abs(self._angular_speed) > 1e-9:
            self._yaw += self._angular_speed * dt

        self._x += self._linear_speed * dt * math.cos(self._yaw)
        self._y += self._linear_speed * dt * math.sin(self._yaw)

    def _publish_open_loop_tf(self):
        """Publish odom to base_footprint without relying on Gazebo odometry."""
        if not self._publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = self._z
        transform.transform.rotation.z = math.sin(self._yaw * 0.5)
        transform.transform.rotation.w = math.cos(self._yaw * 0.5)

        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Run the one-turn spin command node."""
    rclpy.init(args=args)
    node = SpinOnceNode()

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
